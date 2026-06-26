/**
 * @file        jm_proto_ops.c
 * @brief       关节电机协议-业务回调实现(传输无关, 串口/CAN 共用)
 *
 * @author      Dalin (dalin@robot.com)
 * @version     1.0
 * @date        2026-06-23
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-23 | 1.0  | Dalin  | 初始创建   |
 * | 2026-06-25 | 1.1  | Dalin  | 补全 0xC9/0xE2/0xE3/0xF0/0xF1 回调实现 |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        jm_proto_ops_t 全部回调的统一落点。jm_proto_dispatch() 解析出
 *              cmd+payload 后回调到这里, 串口(jm_proto_uart)与 CAN(jm_proto_can)
 *              注入同一份 ops, 故两条链路命令处理完全一致。
 * @note        载荷布局严格依据 Protocol/docs/joint_motor_command_list.csv 与
 *              joint_motor_param_index.csv; 全部小端, 用 jm_proto.h 小端助手解析。
 * @note        线程模型: 本回调在通信线程上下文执行, 控制目标写入 motor_loop 的
 *              sys.motor.cmd 后由电流环 ISR 读取。单生产者(通信)单消费者(ISR)、
 *              且写入的是独立 float 字段, 沿用工程既有无锁约定。
 */
#include <stddef.h>
#include <string.h>

#include "jm_proto_ops.h"
#include "runtime_param.h" /* usr, motor_state_t, motor_param_t, M1 */
#include "motor_param.h"   /* motor_param_init / 字段类型 */
#include "version.h"	   /* HW_/APP_ 版本号 */
#include "motor_loop.h"	   /* motor_loop_get / motor_loop_set_cmd */

/* ============================================================================
 *  1) 控制/模式: CMD 0x00~0xB8  ->  set_mode
 *     CMD 数值即 ctrl_mode_e。先把目标量写入 sys.motor.cmd(motor_cmd_t),
 *     再调 motor_loop_set_cmd() 触发状态机切换; 下游 run_*_control 读取这些目标。
 * ==========================================================================*/

/* 把 set_mode 的目标量写入电机控制核心的指令缓冲(motor_cmd_t) */
static motor_cmd_t *app_motor_cmd(void)
{
	return &motor_loop_get()->sys.motor.cmd;
}

static jm_err_e app_set_mode(uint8_t cmd, const uint8_t *pl, uint16_t len)
{
	motor_cmd_t *mc = app_motor_cmd();

	switch (cmd)
	{
		/* ---- 系统控制 0x00~0x06: 无载荷, 仅切状态 ---- */
		case JM_CMD_IDLE:
		case JM_CMD_HOLD:
		case JM_CMD_BRAKE:
		case JM_CMD_ESTOP:
		case JM_CMD_ENABLE:
		case JM_CMD_DISABLE:
		case JM_CMD_STOP:
			break;

		/* ---- 开环电压 {ud,uq}: 下游用 cmd.torque 作开环电压目标 ---- */
		case JM_CMD_OPEN_LOOP:
			if (len < 8)
				return JM_ERR_LENGTH;
			mc->id = jm_rd_f32(&pl[0]);	   /* ud(暂存, 预留) */
			mc->torque = jm_rd_f32(&pl[4]); /* uq -> 开环电压 */
			break;

		/* ---- 电流环 {id,iq} ---- */
		case JM_CMD_CURRENT:
		case JM_CMD_FIELD_WEAKENING:
			if (len < 8)
				return JM_ERR_LENGTH;
			mc->id = jm_rd_f32(&pl[0]);
			mc->iq = jm_rd_f32(&pl[4]);
			break;

		/* ---- 力矩环 {torque} ---- */
		case JM_CMD_TORQUE:
		case JM_CMD_FORCE_CONTROL:
		case JM_CMD_CONSTANT_FORCE:
			if (len < 4)
				return JM_ERR_LENGTH;
			mc->torque = jm_rd_f32(&pl[0]);
			break;

		/* ---- MIT/阻抗 {pos,vel,kp,kd,tff}: CAN 层已解压成 5*f32 ---- */
		case JM_CMD_MIT:
		case JM_CMD_IMPEDANCE:
			if (len < 20)
				return JM_ERR_LENGTH;
			mc->pos = jm_rd_f32(&pl[0]);
			mc->vel = jm_rd_f32(&pl[4]);
			mc->kp = jm_rd_f32(&pl[8]);
			mc->kd = jm_rd_f32(&pl[12]);
			mc->torque_ff = jm_rd_f32(&pl[16]);
			break;

		/* ---- 速度环 {vel} ---- */
		case JM_CMD_VELOCITY:
		case JM_CMD_SENSORLESS:
			if (len < 4)
				return JM_ERR_LENGTH;
			mc->vel = jm_rd_f32(&pl[0]);
			break;

		/* ---- 位置环 {pos} ---- */
		case JM_CMD_POSITION:
			if (len < 4)
				return JM_ERR_LENGTH;
			mc->pos = jm_rd_f32(&pl[0]);
			break;

		/* ---- 位置+速度前馈 {pos,vel_ff} ---- */
		case JM_CMD_POSITION_VELOCITY:
			if (len < 8)
				return JM_ERR_LENGTH;
			mc->pos = jm_rd_f32(&pl[0]);
			mc->vel = jm_rd_f32(&pl[4]); /* 作速度前馈 */
			break;

		/* ---- 位置+力矩限幅 {pos,tq_lim} ---- */
		case JM_CMD_POSITION_TORQUE:
			if (len < 8)
				return JM_ERR_LENGTH;
			mc->pos = jm_rd_f32(&pl[0]);
			mc->torque = jm_rd_f32(&pl[4]);
			break;

		/* ---- 速度+力矩限幅 {vel,tq_lim} ---- */
		case JM_CMD_VELOCITY_TORQUE:
			if (len < 8)
				return JM_ERR_LENGTH;
			mc->vel = jm_rd_f32(&pl[0]);
			mc->torque = jm_rd_f32(&pl[4]);
			break;

		/* ---- 占空比 {duty}: 下游用 cmd.torque 作占空比目标 ---- */
		case JM_CMD_DUTY_CYCLE:
			if (len < 4)
				return JM_ERR_LENGTH;
			mc->torque = jm_rd_f32(&pl[0]);
			break;

		/* ---- 其余模式(力控/轨迹/特殊/测试/校准/诊断): 暂仅切状态 ----
		 * 这些模式的载荷由各自 run_*_control 处理逻辑后续接管; 当前先保证
		 * 模式切换可达。无法识别的码不在 0x00~0xB8 段(dispatch 已过滤)。*/
		default:
			break;
	}

	/* 触发状态机: cmd 数值与 ctrl_mode_e 一致, 由 process_ctrl_cmd 解释 */
	motor_loop_set_cmd((ctrl_mode_e)cmd);
	return JM_ERR_OK;
}

/* ============================================================================
 *  2) 反馈/状态: CMD 0xC0~0xC8 / 0xD2  ->  get_feedback
 *     从运行时快照 usr.motor_state[M1] 单向读取, 不触碰控制层内部结构。
 * ==========================================================================*/
jm_err_e jm_app_get_feedback(jm_feedback_t *fb)
{
	const motor_state_t *m = &usr.motor_state[M1];

	if (fb == NULL)
	{
		return JM_ERR_STATE_DENY;
	}

	fb->pos = m->motion.position_rad;		/* 输出端多圈位置 rad */
	fb->vel = m->motion.velocity_rad_s;		/* 输出端速度 rad/s */
	fb->torque = m->power.torque_est;		/* 输出端力矩 Nm(估算) */
	fb->id = m->electrical.id_meas;			/* d轴电流 A */
	fb->iq = m->electrical.iq_meas;			/* q轴电流 A */
	fb->ia = m->electrical.ia;				/* A 相电流 A */
	fb->ib = m->electrical.ib;				/* B 相电流 A */
	fb->ic = m->electrical.ic;				/* C 相电流 A */
	fb->vbus = m->power.v_bus;				/* 母线电压 V */
	fb->ibus = m->power.i_bus;				/* 母线电流 A */
	fb->temp_fet = m->thermal.temp_fet;		/* 功率管温度 ℃ */
	fb->temp_motor = m->thermal.temp_motor; /* 电机温度 ℃ */
	fb->multiturn = m->motion.multiturn;	/* 多圈计数 */
	fb->single = m->motion.single_turn_rad; /* 单圈位置 rad */
	fb->fault_mask = m->fault.fault_mask;	/* 故障掩码 */
	fb->warn_mask = m->fault.warn_mask;		/* 警告掩码 */
	fb->top_fsm = (uint8_t)usr.fsm.motor_fsm[M1];
	fb->run_state = (uint8_t)m->run_mode;
	fb->ctrl_mode = (uint8_t)usr.fsm.motor_mode[M1];
	fb->enable = m->enable_motor ? 1u : 0u;
	return JM_ERR_OK;
}

static jm_err_e app_get_feedback(jm_feedback_t *fb)
{
	return jm_app_get_feedback(fb);
}

/* ============================================================================
 *  3) 参数读写: CMD 0xE0~0xE5  ->  param_read / param_write / param_save / param_reset
 *     param_id 与 joint_motor_param_index.csv 一一对应。用偏移表把 param_id 映射到
 *     motor_param_t 内的字段(类型+偏移+字节数), 读写直接按内存小端拷贝。
 *     (Cortex-M 小端, 与协议小端一致, 无需逐字段转换。)
 * ==========================================================================*/

typedef struct
{
	uint16_t offset; /* 字段在 motor_param_t 内的字节偏移 */
	uint8_t type;	 /* jm_param_type_e */
	uint8_t size;	 /* 字段字节数 */
} param_desc_t;

/* 子结构字段 -> 全局偏移; size 由类型决定, 避免与 type 不一致 */
#define PT_SZ(t) ((t) == JM_PT_U8 || (t) == JM_PT_I8 ? 1 : \
				  (t) == JM_PT_U16 || (t) == JM_PT_I16 ? 2 : \
				  (t) == JM_PT_STR ? 16 : 4)

#define PARAM_ENT(grp, subtype, field, ptype)                                   \
	{                                                                           \
		(uint16_t)(offsetof(motor_param_t, grp) + offsetof(subtype, field)),    \
			(uint8_t)(ptype), (uint8_t)PT_SZ(ptype)                             \
	}

/* 索引即 param_id(0~82), 顺序严格对齐 joint_motor_param_index.csv */
static const param_desc_t s_param_tbl[MOTOR_PARAM_PARAM_COUNT] = {
	/* 0~1 实例标识 */
	PARAM_ENT(motor_instance, motor_instance_t, motor_id, JM_PT_U8),
	PARAM_ENT(motor_instance, motor_instance_t, motor_name, JM_PT_STR),
	/* 2~19 电机本体 */
	PARAM_ENT(motor_base, motor_base_t, r, JM_PT_F32),
	PARAM_ENT(motor_base, motor_base_t, ld, JM_PT_F32),
	PARAM_ENT(motor_base, motor_base_t, lq, JM_PT_F32),
	PARAM_ENT(motor_base, motor_base_t, flux, JM_PT_F32),
	PARAM_ENT(motor_base, motor_base_t, kt, JM_PT_F32),
	PARAM_ENT(motor_base, motor_base_t, pole_pairs, JM_PT_U8),
	PARAM_ENT(motor_base, motor_base_t, rated_current, JM_PT_F32),
	PARAM_ENT(motor_base, motor_base_t, peak_current, JM_PT_F32),
	PARAM_ENT(motor_base, motor_base_t, max_speed, JM_PT_F32),
	PARAM_ENT(motor_base, motor_base_t, dead_time_ns, JM_PT_F32),
	PARAM_ENT(motor_base, motor_base_t, rated_voltage, JM_PT_F32),
	PARAM_ENT(motor_base, motor_base_t, rated_speed_rpm, JM_PT_F32),
	PARAM_ENT(motor_base, motor_base_t, rated_torque, JM_PT_F32),
	PARAM_ENT(motor_base, motor_base_t, peak_torque, JM_PT_F32),
	PARAM_ENT(motor_base, motor_base_t, inertia, JM_PT_F32),
	PARAM_ENT(motor_base, motor_base_t, ke, JM_PT_F32),
	PARAM_ENT(motor_base, motor_base_t, pwm_freq_hz, JM_PT_U32),
	PARAM_ENT(motor_base, motor_base_t, foc_freq_hz, JM_PT_U32),
	/* 20~23 减速器 */
	PARAM_ENT(gearbox_param, gearbox_param_t, gear_ratio, JM_PT_F32),
	PARAM_ENT(gearbox_param, gearbox_param_t, gear_efficiency, JM_PT_F32),
	PARAM_ENT(gearbox_param, gearbox_param_t, output_torque_const, JM_PT_F32),
	PARAM_ENT(gearbox_param, gearbox_param_t, gear_backlash, JM_PT_F32),
	/* 24~31 编码器 */
	PARAM_ENT(encoder_param, encoder_param_t, enc_lines, JM_PT_U32),
	PARAM_ENT(encoder_param, encoder_param_t, enc_direction, JM_PT_I8),
	PARAM_ENT(encoder_param, encoder_param_t, enc_offset, JM_PT_I32),
	PARAM_ENT(encoder_param, encoder_param_t, elec_angle_bias, JM_PT_F32),
	PARAM_ENT(encoder_param, encoder_param_t, pos_filter_alpha, JM_PT_F32),
	PARAM_ENT(encoder_param, encoder_param_t, enc_type, JM_PT_U8),
	PARAM_ENT(encoder_param, encoder_param_t, enc_auto_calib, JM_PT_U8),
	PARAM_ENT(encoder_param, encoder_param_t, speed_obs_gain, JM_PT_F32),
	/* 32~35 位置限位 */
	PARAM_ENT(position_limit, position_limit_t, multiturn_enable, JM_PT_U8),
	PARAM_ENT(position_limit, position_limit_t, pos_min_limit, JM_PT_F32),
	PARAM_ENT(position_limit, position_limit_t, pos_max_limit, JM_PT_F32),
	PARAM_ENT(position_limit, position_limit_t, limit_sw_enable, JM_PT_U8),
	/* 36~40 回零 */
	PARAM_ENT(homing_param, homing_param_t, homing_method, JM_PT_U8),
	PARAM_ENT(homing_param, homing_param_t, homing_speed_fast, JM_PT_F32),
	PARAM_ENT(homing_param, homing_param_t, homing_speed_slow, JM_PT_F32),
	PARAM_ENT(homing_param, homing_param_t, homing_offset, JM_PT_F32),
	PARAM_ENT(homing_param, homing_param_t, homing_current, JM_PT_F32),
	/* 41~52 电流环 */
	PARAM_ENT(current_loop, current_loop_t, current_kp_d, JM_PT_F32),
	PARAM_ENT(current_loop, current_loop_t, current_ki_d, JM_PT_F32),
	PARAM_ENT(current_loop, current_loop_t, current_kp_q, JM_PT_F32),
	PARAM_ENT(current_loop, current_loop_t, current_ki_q, JM_PT_F32),
	PARAM_ENT(current_loop, current_loop_t, current_integral_limit, JM_PT_F32),
	PARAM_ENT(current_loop, current_loop_t, decoupling_gain, JM_PT_F32),
	PARAM_ENT(current_loop, current_loop_t, deadtime_comp_v, JM_PT_F32),
	PARAM_ENT(current_loop, current_loop_t, pwm_max_duty, JM_PT_F32),
	PARAM_ENT(current_loop, current_loop_t, current_bandwidth_hz, JM_PT_F32),
	PARAM_ENT(current_loop, current_loop_t, current_filter_alpha, JM_PT_F32),
	PARAM_ENT(current_loop, current_loop_t, d_feedforward_gain, JM_PT_F32),
	PARAM_ENT(current_loop, current_loop_t, q_feedforward_gain, JM_PT_F32),
	/* 53~68 位置速度环 */
	PARAM_ENT(position_loop, position_loop_t, speed_kp, JM_PT_F32),
	PARAM_ENT(position_loop, position_loop_t, speed_ki, JM_PT_F32),
	PARAM_ENT(position_loop, position_loop_t, speed_integral_limit, JM_PT_F32),
	PARAM_ENT(position_loop, position_loop_t, velocity_ff_gain, JM_PT_F32),
	PARAM_ENT(position_loop, position_loop_t, accel_ff_gain, JM_PT_F32),
	PARAM_ENT(position_loop, position_loop_t, position_kp, JM_PT_F32),
	PARAM_ENT(position_loop, position_loop_t, position_integral_limit, JM_PT_F32),
	PARAM_ENT(position_loop, position_loop_t, friction_coulomb, JM_PT_F32),
	PARAM_ENT(position_loop, position_loop_t, friction_viscous, JM_PT_F32),
	PARAM_ENT(position_loop, position_loop_t, notch_freq_hz, JM_PT_F32),
	PARAM_ENT(position_loop, position_loop_t, notch_width_hz, JM_PT_F32),
	PARAM_ENT(position_loop, position_loop_t, notch_depth_db, JM_PT_F32),
	PARAM_ENT(position_loop, position_loop_t, notch_enable, JM_PT_U8),
	PARAM_ENT(position_loop, position_loop_t, speed_bandwidth_hz, JM_PT_F32),
	PARAM_ENT(position_loop, position_loop_t, speed_filter_alpha, JM_PT_F32),
	PARAM_ENT(position_loop, position_loop_t, position_bandwidth_hz, JM_PT_F32),
	/* 69~71 阻抗控制 */
	PARAM_ENT(impedance_ctrl, impedance_ctrl_t, impedance_kp, JM_PT_F32),
	PARAM_ENT(impedance_ctrl, impedance_ctrl_t, impedance_kd, JM_PT_F32),
	PARAM_ENT(impedance_ctrl, impedance_ctrl_t, iq_max, JM_PT_F32),
	/* 72~74 热模型 */
	PARAM_ENT(thermal_model, thermal_model_t, thermal_resistance, JM_PT_F32),
	PARAM_ENT(thermal_model, thermal_model_t, thermal_time_const, JM_PT_F32),
	PARAM_ENT(thermal_model, thermal_model_t, derating_temp_start, JM_PT_F32),
	/* 75~82 保护 */
	PARAM_ENT(protection_param, protection_param_t, protect_over_current, JM_PT_F32),
	PARAM_ENT(protection_param, protection_param_t, protect_over_voltage, JM_PT_F32),
	PARAM_ENT(protection_param, protection_param_t, protect_under_voltage, JM_PT_F32),
	PARAM_ENT(protection_param, protection_param_t, protect_over_speed, JM_PT_F32),
	PARAM_ENT(protection_param, protection_param_t, protect_over_temp, JM_PT_F32),
	PARAM_ENT(protection_param, protection_param_t, protect_under_temp, JM_PT_F32),
	PARAM_ENT(protection_param, protection_param_t, protect_pos_error, JM_PT_I32),
	PARAM_ENT(protection_param, protection_param_t, protect_enable_mask, JM_PT_U32),
};

static jm_err_e app_param_read(uint16_t param_id, uint8_t *value,
							   uint8_t *out_type, uint8_t *out_len)
{
	const param_desc_t *d;
	const uint8_t *base = (const uint8_t *)&usr.motor_param[M1];

	if (param_id >= MOTOR_PARAM_PARAM_COUNT)
	{
		return JM_ERR_BAD_PARAM_ID;
	}
	d = &s_param_tbl[param_id];
	memcpy(value, base + d->offset, d->size); /* 内存即小端, 直接拷出 */
	*out_type = d->type;
	*out_len = d->size;
	return JM_ERR_OK;
}

static jm_err_e app_param_write(uint16_t param_id, const uint8_t *value, uint8_t len)
{
	const param_desc_t *d;
	uint8_t *base = (uint8_t *)&usr.motor_param[M1];

	if (param_id >= MOTOR_PARAM_PARAM_COUNT)
	{
		return JM_ERR_BAD_PARAM_ID;
	}
	d = &s_param_tbl[param_id];
	/* 字符串允许短于 16(截断存入), 其余类型长度须精确匹配 */
	if (d->type == JM_PT_STR)
	{
		if (len == 0u || len > d->size)
			return JM_ERR_LENGTH;
		memset(base + d->offset, 0, d->size);
		memcpy(base + d->offset, value, len);
	}
	else
	{
		if (len != d->size)
			return JM_ERR_LENGTH;
		memcpy(base + d->offset, value, len);
	}
	return JM_ERR_OK;
}

/* 默认参数持久化: 弱实现, 仅做范围校验, 不落 Flash。
 * 接入 Flash 驱动后在驱动层提供同名强符号即可覆盖。*/
#if defined(__GNUC__) || defined(__clang__)
__attribute__((weak))
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
__weak
#endif
int jm_app_param_storage_save(const motor_param_t *cfg)
{
	return motor_param_validate(cfg); /* 0=全部通过, 否则首个越界 param_id(>0) */
}

static jm_err_e app_param_save(void)
{
	int rc = jm_app_param_storage_save(&usr.motor_param[M1]);
	if (rc == 0)
	{
		return JM_ERR_OK;
	}
	/* 校验越界(>0) -> 越界; 负值(如 -EINVAL) 或写失败 -> Flash 失败 */
	return (rc > 0) ? JM_ERR_OUT_OF_RANGE : JM_ERR_FLASH;
}

static jm_err_e app_param_reset(uint16_t param_id)
{
	motor_param_t def;

	/* 0xFFFF: 全部恢复默认 */
	if (param_id == 0xFFFFu)
	{
		return (motor_param_init(&usr.motor_param[M1]) == 0) ? JM_ERR_OK : JM_ERR_FLASH;
	}
	if (param_id >= MOTOR_PARAM_PARAM_COUNT)
	{
		return JM_ERR_BAD_PARAM_ID;
	}
	/* 单参数恢复: 取默认实例对应字段覆盖当前值 */
	if (motor_param_init(&def) != 0)
	{
		return JM_ERR_FLASH;
	}
	{
		const param_desc_t *d = &s_param_tbl[param_id];
		uint8_t *dst = (uint8_t *)&usr.motor_param[M1] + d->offset;
		const uint8_t *src = (const uint8_t *)&def + d->offset;
		memcpy(dst, src, d->size);
	}
	return JM_ERR_OK;
}

/* ---- 批量读参数 0xE2: 复用 s_param_tbl, 按 [type:u8][value] 顺序打包 ----
 * 应答体布局: [start_id:u16][count:u8][[type:u8][value]...] (count 为实际读到的个数,
 * 越界或超单帧容量时截断; 主机据每个 type 的字节数顺序解析至帧尾)。*/
static jm_err_e app_param_read_bulk(uint16_t start_id, uint16_t count,
									uint8_t *out, uint16_t *out_len)
{
	const uint8_t *base = (const uint8_t *)&usr.motor_param[M1];
	uint16_t n = 0;
	uint16_t i;
	uint8_t actual = 0;

	if (start_id >= MOTOR_PARAM_PARAM_COUNT)
	{
		return JM_ERR_BAD_PARAM_ID;
	}
	/* 头部: start_id(2) + count(1), count 字节稍后回填 */
	jm_wr_u16(&out[0], start_id);
	n = 3;

	for (i = 0; i < count; i++)
	{
		uint16_t pid = (uint16_t)(start_id + i);
		const param_desc_t *d;
		/* 越界则提前结束, 返回实际读到的个数 */
		if (pid >= MOTOR_PARAM_PARAM_COUNT)
		{
			break;
		}
		/* 预留 type(1) + 最大 value(STR=16); 超单帧容量则截断 */
		if ((uint32_t)n + 1u + 16u > JM_PAYLOAD_MAX)
		{
			break;
		}
		d = &s_param_tbl[pid];
		out[n++] = d->type;
		memcpy(&out[n], base + d->offset, d->size);
		n += d->size;
		actual++;
	}
	out[2] = actual; /* 回填实际参数个数 */
	*out_len = n;
	return JM_ERR_OK;
}

/* ---- 批量写参数 0xE3: values 为按参数表类型逐个拼接的原始字节 ----
 * 每个值长度由 (start_id+i) 的类型决定(字符串亦按完整 size=16); 不足则 LENGTH。
 * 任一参数越界则整体失败回 BAD_PARAM_ID(已写入的前序值不回滚, 由主机重读校正)。*/
static jm_err_e app_param_write_bulk(uint16_t start_id, uint16_t count,
									 const uint8_t *values, uint16_t len)
{
	uint8_t *base = (uint8_t *)&usr.motor_param[M1];
	uint16_t off = 0;
	uint16_t i;

	for (i = 0; i < count; i++)
	{
		uint16_t pid = (uint16_t)(start_id + i);
		const param_desc_t *d;
		if (pid >= MOTOR_PARAM_PARAM_COUNT)
		{
			return JM_ERR_BAD_PARAM_ID;
		}
		d = &s_param_tbl[pid];
		if ((uint32_t)off + d->size > len)
		{
			return JM_ERR_LENGTH;
		}
		memcpy(base + d->offset, &values[off], d->size);
		off += d->size;
	}
	return JM_ERR_OK;
}

/* ============================================================================
 *  4) 设备信息: CMD 0xD0/0xD1  ->  get_dev_info / get_dev_name
 * ==========================================================================*/

/* 版本号打包: [major][minor][patch][build] -> u32(高位在前) */
#define JM_VER_PACK(maj, min, pat, bld) \
	(((uint32_t)(maj) << 24) | ((uint32_t)(min) << 16) | ((uint32_t)(pat) << 8) | (uint32_t)(bld))

static jm_err_e app_get_dev_info(uint32_t *hw_ver, uint32_t *fw_ver, uint8_t uid[12])
{
	*hw_ver = JM_VER_PACK(HW_VERSION_MAJOR, HW_VERSION_MINOR, HW_VERSION_PATCH, HW_VERSION_BUILD);
	*fw_ver = JM_VER_PACK(APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_PATCH, APP_VERSION_BUILD);
	memcpy(uid, usr.sys.info.device_uid, sizeof(usr.sys.info.device_uid));
	return JM_ERR_OK;
}

static const char *app_get_dev_name(void)
{
	const char *name = motor_param_get_motor_name(&usr.motor_param[M1]);
	if (name == NULL || name[0] == '\0')
	{
		return "JointMotor"; /* 未命名时给默认名 */
	}
	return name;
}

/* ============================================================================
 *  5) 同步遥测订阅: CMD 0xCB  ->  set_telemetry
 *     存上报总开关/订阅掩码/周期, 供绑定层(串口/CAN 周期帧)读取后自行打包上报。
 *     enable=1 启动周期上报, enable=0 停止; 数据帧 0xCA 周期主动推送, 不逐帧应答。
 * ==========================================================================*/
static uint8_t s_tlm_enable = 0u;	  /* 周期上报总开关: 0=停止, 1=启动 */
static uint16_t s_tlm_mask = 0xFFFFu; /* 默认订阅全部组 */
static uint16_t s_tlm_period_ms = 0u; /* 0 表示沿用绑定层默认周期 */

static jm_err_e app_set_telemetry(uint8_t enable, uint16_t mask, uint16_t period_ms)
{
	s_tlm_enable = enable ? 1u : 0u;
	s_tlm_mask = mask;
	if (period_ms != 0u)
	{
		s_tlm_period_ms = period_ms;
	}
	return JM_ERR_OK;
}

uint8_t jm_app_telemetry_enabled(void)
{
	return s_tlm_enable;
}

uint16_t jm_app_telemetry_mask(void)
{
	return s_tlm_mask;
}

uint16_t jm_app_telemetry_period_ms(void)
{
	return s_tlm_period_ms;
}

/* ============================================================================
 *  6) 调试通道: CMD 0xC9  ->  get_debug
 *     直接读 jm_dbg[JM_DBG_CH](任意处可写的观测点), 与 0xCA 遥测 DEBUG 组同源。
 * ==========================================================================*/
static jm_err_e app_get_debug(float *out, uint8_t *out_count, uint8_t max_count)
{
	uint8_t i;
	uint8_t cnt = JM_DBG_CH;

	if (cnt > max_count)
	{
		cnt = max_count;
	}
	for (i = 0; i < cnt; i++)
	{
		out[i] = jm_dbg[i];
	}
	*out_count = cnt;
	return JM_ERR_OK;
}

/* ============================================================================
 *  7) CAN 管理: CMD 0xF0/0xF1  ->  set_can_id / set_baudrate
 *     两者均写入 RAM 配置, 持久化由主机显式发 0xE4 完成, 重启后由 CAN 绑定层加载生效。
 * ==========================================================================*/
static uint8_t s_can_baud_code = 0u; /* 0=1M(默认) 1=500K 2=250K 3=125K */

static jm_err_e app_set_can_id(uint8_t new_id)
{
	/* 写入参数表 motor_id; 范围 1~127 已由 dispatch 校验。
	 * CAN 滤波地址在绑定层初始化时读取, 故重启后生效。*/
	return (motor_param_set_motor_id(&usr.motor_param[M1], new_id) == 0)
			   ? JM_ERR_OK
			   : JM_ERR_OUT_OF_RANGE;
}

static jm_err_e app_set_baudrate(uint8_t baud_code)
{
	s_can_baud_code = baud_code;
	return JM_ERR_OK;
}

uint8_t jm_app_can_baudrate(void)
{
	return s_can_baud_code;
}

/* ============================================================================
 *  回调集单例
 * ==========================================================================*/
static const jm_proto_ops_t s_app_ops = {
	.set_mode = app_set_mode,
	.get_feedback = app_get_feedback,
	.param_read = app_param_read,
	.param_write = app_param_write,
	.param_save = app_param_save,
	.param_reset = app_param_reset,
	.get_dev_info = app_get_dev_info,
	.get_dev_name = app_get_dev_name,
	.set_telemetry = app_set_telemetry,
	.get_debug = app_get_debug,
	.param_read_bulk = app_param_read_bulk,
	.param_write_bulk = app_param_write_bulk,
	.set_can_id = app_set_can_id,
	.set_baudrate = app_set_baudrate,
};

const jm_proto_ops_t *jm_app_ops_get(void)
{
	return &s_app_ops;
}
