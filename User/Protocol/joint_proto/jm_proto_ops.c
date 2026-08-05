/**
 * @file        jm_proto_ops.c
 * @brief       关节电机协议-业务回调实现(传输无关, 串口/CAN 共用)
 *
 * @author      Dalin (dalinyy@163.com)
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
#include "runtime_param.h"      /* usr, motor_state_t, motor_param_t, M1 */
#include "motor_param.h"        /* motor_param_init / 字段类型 */
#include "motor_profile.h"      /* motor_profile_apply_param/info 覆盖电机电气身份 */
#include "version.h"            /* HW_/APP_ 版本号 */
#include "motor_loop.h"         /* motor_loop_get / motor_loop_set_cmd */
#include "motor_info.h"         /* motor_info_t / motor_info_init / motor_info_dispatch_read/write */
#include "motor_info_storage.h" /* motor_info_storage_get: 获取全局 motor_info 句柄 */
#include "motor_info_calib.h"   /* motor_info_calib_reset_for_recalibration: 0xEC 命令实现 */
#include "calib_mgr.h"          /* 标定管理器 start/poll/abort/get_status */
#include "motor_pid_autotune.h" /* motor_pid_autotune_apply: 零极点对消法理论估计 */
#include "motor_pid_load.h"     /* motor_pid_set_source / motor_pid_reload: 三环独立 source */
#include "main.h"               /* HAL_GetTick (速率限制) */
#if defined(USE_DEV_LED)
#include "led_manager.h"
#endif
#if defined(USE_DEV_COMMUN_CAN)
#include "dev_commun_can.h"     /* dev_commun_can: 0xF3 SET_FD_MODE 切换运行期FD模式 */
#include "jm_proto_can.h"       /* jm_proto_can_set_fd_mode */
#endif

#if defined(USE_DEV_FLASH)
/* 强实现位于 motor_info_storage.c, 本文件后部保留弱实现供无存储后端时覆盖。 */
motor_info_storage_status_t jm_app_motor_info_storage_save(const motor_info_t *cfg);
#endif

/* ============================================================================
 * 命令速率限制 (防 DoS / Flash 寿命损耗)
 *   对危险/高开销命令按类别设最小间隔, 超频返回 NACK(RATE_LIMIT)
 *   开发期默认关闭 (JM_CAN_AUTH_ENABLE=0 时速率限制也关闭, 便于调试)
 * ==========================================================================*/
#if defined(JM_RATE_LIMIT_ENABLE) && (JM_RATE_LIMIT_ENABLE == 1)
#define JM_RATE_MIN_INTERVAL_PARAM_WRITE_MS   100u  /* 0xE1 PARAM_WRITE: 100ms (10Hz) */
#define JM_RATE_MIN_INTERVAL_PARAM_SAVE_MS    1000u /* 0xE4 PARAM_SAVE: 1s (防 Flash 擟写) */
#define JM_RATE_MIN_INTERVAL_CALIB_MS         2000u /* 0x90~0x96 CALIB: 2s (防并发启动) */
#define JM_RATE_MIN_INTERVAL_MOTOR_INFO_W_MS  100u  /* 0xE7/0xE9 MOTOR_INFO_WRITE: 100ms */
#define JM_RATE_MIN_INTERVAL_MOTOR_INFO_S_MS  1000u /* 0xEA MOTOR_INFO_SAVE: 1s */

static uint32_t s_last_tick_param_write = 0;
static uint32_t s_last_tick_param_save = 0;
static uint32_t s_last_tick_calib = 0;
static uint32_t s_last_tick_motor_info_w = 0;
static uint32_t s_last_tick_motor_info_s = 0;

/* 检查速率限制: 返回 1=允许, 0=拒绝(超频) */
static int jm_rate_check(uint32_t *last_tick, uint32_t min_interval_ms)
{
	uint32_t now = HAL_GetTick();
	uint32_t elapsed = now - *last_tick;
	if (elapsed < min_interval_ms)
	{
		return 0; /* 超频 */
	}
	*last_tick = now;
	return 1; /* 允许 */
}

#define JM_RATE_CHECK(last_tick_ptr, min_ms)  jm_rate_check((last_tick_ptr), (min_ms))
#else
#define JM_RATE_CHECK(last_tick_ptr, min_ms)  (1) /* 速率限制关闭, 始终允许 */
#endif /* JM_RATE_LIMIT_ENABLE */

/* ============================================================================
 * 鉴权令牌 (开发期关闭, 量产期启用)
 *   危险命令前 4 字节必须匹配 JM_CAN_AUTH_TOKEN, 否则 NACK(UNAUTHORIZED)
 *   适用命令(实际仅 0xE1 已实现 JM_AUTH_CHECK, 其余为 TODO):
 *     0xE1 PARAM_WRITE        ✓ 已实现
 *     0xE4 PARAM_SAVE         ✗ TODO
 *     0xF0 SET_CAN_ID         ✗ TODO
 *     0xF1 SET_BAUDRATE       ✗ TODO
 *     0x96 CALIB_LEVEL7       ✗ TODO
 *     0xEA MOTOR_INFO_SAVE    ✗ TODO
 * ==========================================================================*/
#if defined(JM_CAN_AUTH_ENABLE) && (JM_CAN_AUTH_ENABLE == 1)
#define JM_AUTH_CHECK(value_ptr, len_var)  jm_auth_check((value_ptr), (len_var))
static int jm_auth_check(const uint8_t *value, uint16_t len)
{
	if (len < 4)
		return 0; /* 令牌缺失 */
	uint32_t token = jm_rd_u32(value);
	return (token == JM_CAN_AUTH_TOKEN) ? 1 : 0;
}
/* 鉴权通过后, 跳过前 4 字节令牌, 实际载荷从 value+4 开始 */
#define JM_AUTH_SKIP(value_ptr, len_var) \
	do { (value_ptr) += 4; (len_var) -= 4; } while (0)
#else
#define JM_AUTH_CHECK(value_ptr, len_var)  (1) /* 鉴权关闭, 始终通过 */
#define JM_AUTH_SKIP(value_ptr, len_var)   ((void)0) /* 无令牌跳过 */
#endif /* JM_CAN_AUTH_ENABLE */

/* ============================================================================
 *  1) 控制/模式: CMD 0x00~0xB8  ->  set_mode
 *     CMD 数值即 ctrl_mode_e。先把目标量写入 sys.motor.cmd(motor_cmd_t),
 *     再调 motor_loop_set_cmd() 触发状态机切换; 下游 run_*_control 读取这些目标。
 * ==========================================================================*/

/* 前向声明: app_param_save 定义在 0xE4 处理段, 此处 0xB3 需提前调用 */
static jm_err_e app_param_save(void);

/* 把 set_mode 的目标量写入电机控制核心的指令缓冲(motor_cmd_t) */
static motor_cmd_t *app_motor_cmd(void)
{
	return &motor_loop_get()->sys.motor.cmd;
}

static int app_float_is_finite(float value)
{
	uint32_t bits;
	memcpy(&bits, &value, sizeof(bits));
	return (bits & 0x7F800000u) != 0x7F800000u;
}

static int app_mode_is_supported(uint8_t cmd)
{
	switch (cmd)
	{
		case JM_CMD_IDLE:
		case JM_CMD_HOLD:
		case JM_CMD_BRAKE:
		case JM_CMD_ESTOP:
		case JM_CMD_ENABLE:
		case JM_CMD_DISABLE:
		case JM_CMD_STOP:
		case JM_CMD_OPEN_LOOP:
		case JM_CMD_CURRENT:
		case JM_CMD_TORQUE:
		case JM_CMD_MIT:
		case JM_CMD_VELOCITY:
		case JM_CMD_POSITION:
		case JM_CMD_POSITION_VELOCITY:
		case JM_CMD_POSITION_TORQUE:
		case JM_CMD_VELOCITY_TORQUE:
		case JM_CMD_DUTY_CYCLE:
		case JM_CMD_CALIB_LEVEL1:
		case JM_CMD_CALIB_LEVEL2:
		case JM_CMD_CALIB_LEVEL3:
		case JM_CMD_CALIB_LEVEL4:
		case JM_CMD_CALIB_LEVEL5:
		case JM_CMD_CALIB_LEVEL6:
		case JM_CMD_CALIB_LEVEL7:
		case JM_CMD_CALIB_ABORT:
		case JM_CMD_CLEAR_FAULT:
			return 1;
		default:
			return 0;
	}
}

static jm_err_e app_set_mode(uint8_t cmd, const uint8_t *pl, uint16_t len)
{
	motor_cmd_t *mc = app_motor_cmd();
	motor_cmd_t next;
	jm_err_e ret = JM_ERR_OK;
	uint32_t primask;

	if (!app_mode_is_supported(cmd))
		return JM_ERR_NOT_SUPPORTED;

	/* 双通道临界区: 保护 motor_cmd 字段写入 + 状态机切换的原子性,
	 * 防止 UART(通信线程) 和 CAN(ISR) 并发调用 app_set_mode 导致状态混乱。
	 * 用 __get_PRIMASK/__set_PRIMASK 保持中断原有使能状态。*/
	primask = __get_PRIMASK();
	__disable_irq();
	next = *mc;

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
			if (len < 8) { ret = JM_ERR_LENGTH; goto done; }
			next.id = jm_rd_f32(&pl[0]);     /* ud(暂存, 预留) */
			next.torque = jm_rd_f32(&pl[4]); /* uq -> 开环电压 */
			break;

		/* ---- 电流环 {id,iq} ---- */
		case JM_CMD_CURRENT:
		case JM_CMD_FIELD_WEAKENING:
			if (len < 8) { ret = JM_ERR_LENGTH; goto done; }
			next.id = jm_rd_f32(&pl[0]);
			next.iq = jm_rd_f32(&pl[4]);
			break;

		/* ---- 力矩环 {torque} ---- */
		case JM_CMD_TORQUE:
		case JM_CMD_FORCE_CONTROL:
		case JM_CMD_CONSTANT_FORCE:
			if (len < 4) { ret = JM_ERR_LENGTH; goto done; }
			next.torque = jm_rd_f32(&pl[0]);
			break;

		/* ---- MIT/阻抗 {pos,vel,kp,kd,tff}: CAN 层已解压成 5*f32 ---- */
		case JM_CMD_MIT:
		case JM_CMD_IMPEDANCE:
			if (len < 20) { ret = JM_ERR_LENGTH; goto done; }
			next.pos = jm_rd_f32(&pl[0]);
			next.vel = jm_rd_f32(&pl[4]);
			next.kp = jm_rd_f32(&pl[8]);
			next.kd = jm_rd_f32(&pl[12]);
			next.torque_ff = jm_rd_f32(&pl[16]);
			break;

		/* ---- 速度环 {vel} ---- */
		case JM_CMD_VELOCITY:
		case JM_CMD_SENSORLESS:
			if (len < 4) { ret = JM_ERR_LENGTH; goto done; }
			next.vel = jm_rd_f32(&pl[0]);
			break;

		/* ---- 位置环 {pos} ---- */
		case JM_CMD_POSITION:
			if (len < 4) { ret = JM_ERR_LENGTH; goto done; }
			next.pos = jm_rd_f32(&pl[0]);
			break;

		/* ---- 位置+速度前馈 {pos,vel_ff} ---- */
		case JM_CMD_POSITION_VELOCITY:
			if (len < 8) { ret = JM_ERR_LENGTH; goto done; }
			next.pos = jm_rd_f32(&pl[0]);
			next.vel = jm_rd_f32(&pl[4]); /* 作速度前馈 */
			break;

		/* ---- 位置+力矩限幅 {pos,tq_lim} ---- */
		case JM_CMD_POSITION_TORQUE:
			if (len < 8) { ret = JM_ERR_LENGTH; goto done; }
			next.pos = jm_rd_f32(&pl[0]);
			next.torque = jm_rd_f32(&pl[4]);
			break;

		/* ---- 速度+力矩限幅 {vel,tq_lim} ---- */
		case JM_CMD_VELOCITY_TORQUE:
			if (len < 8) { ret = JM_ERR_LENGTH; goto done; }
			next.vel = jm_rd_f32(&pl[0]);
			next.torque = jm_rd_f32(&pl[4]);
			break;

		/* ---- 占空比 {duty}: 下游用 cmd.torque 作占空比目标 ---- */
		case JM_CMD_DUTY_CYCLE:
			if (len < 4) { ret = JM_ERR_LENGTH; goto done; }
			next.torque = jm_rd_f32(&pl[0]);
			break;

			/* ---- 标定启动 0x90-0x96: payload[0]=子模式 ---- */
		case JM_CMD_CALIB_LEVEL1:
		case JM_CMD_CALIB_LEVEL2:
		case JM_CMD_CALIB_LEVEL3:
		case JM_CMD_CALIB_LEVEL4:
		case JM_CMD_CALIB_LEVEL5:
		case JM_CMD_CALIB_LEVEL6:
		case JM_CMD_CALIB_LEVEL7:
		{
			if (len < 1) { ret = JM_ERR_LENGTH; goto done; }
			/* 速率限制: 0x90~0x96 CALIB 2s 间隔 (防并发启动) */
			if (!JM_RATE_CHECK(&s_last_tick_calib, JM_RATE_MIN_INTERVAL_CALIB_MS))
			{ ret = JM_ERR_RATE_LIMIT; goto done; }

			uint8_t level = cmd - JM_CMD_CALIB_LEVEL1 + 1;
			uint8_t submode = pl[0];
			if (!calib_mgr_start(level, submode))
			{
				/* 区分失败原因：已在标定中 → BUSY，前置依赖未完成 → STATE_DENY，
			 * 其余（submode 越界/不支持）→ OUT_OF_RANGE */
				calib_status_t st = calib_mgr_get_status();
				if (st.state == CALIB_STATE_RUNNING)
				{ ret = JM_ERR_CALIB_BUSY; goto done; }      /* NACK(0x0A) 已在标定中 */
				if (st.fail_reason == CALIB_FAIL_DEP_NOT_MET)
				{ ret = JM_ERR_STATE_DENY; goto done; }      /* NACK(0x03) 前置标定未完成 */
				ret = JM_ERR_OUT_OF_RANGE;                   /* NACK(0x02) submode 不合法 */
				goto done;
			}
			break;                            /* 继续走 motor_loop_set_cmd 进入 CALIB 态 */
		}

		/* ---- 标定进度查询 0x97: 由 jm_proto.c dispatch 直接返回 8 字节详细状态 ACK,
	 *       不进入本函数, 此处不再处理 ---- */

		/* ---- 标定中止 0x98: 不切状态，直接返回 ---- */
		case JM_CMD_CALIB_ABORT:
		{
			calib_mgr_abort();
			ret = JM_ERR_OK; /* ACK */
			goto done;
		}

		/* ---- 保存配置 0xB3: 与 0xE4 PARAM_SAVE 等价, 调用 app_param_save 写 Flash ---- */
		case JM_CMD_SAVE_CONFIG:
		{
			ret = app_param_save();
			goto done;
		}

		/* ---- 其余模式(力控/轨迹/特殊/测试/诊断): 暂仅切状态 ----
	 * 这些模式的载荷由各自 run_*_control 处理逻辑后续接管; 当前先保证
	 * 模式切换可达。无法识别的码不在 0x00~0xB8 段(dispatch 已过滤)。*/
		default:
			break;
	}

	if (!app_float_is_finite(next.pos) || !app_float_is_finite(next.vel) ||
		!app_float_is_finite(next.torque) || !app_float_is_finite(next.id) ||
		!app_float_is_finite(next.iq) || !app_float_is_finite(next.kp) ||
		!app_float_is_finite(next.kd) || !app_float_is_finite(next.torque_ff) ||
		!app_float_is_finite(next.vel_ff))
	{
		ret = JM_ERR_OUT_OF_RANGE;
		goto done;
	}
	*mc = next;
	motor_loop_set_cmd((ctrl_mode_e)cmd);
	ret = JM_ERR_OK;

done:
	__set_PRIMASK(primask);
	return ret;
}

/* ============================================================================
 *  1b) PID 理论估计: CMD 0xA0  ->  pid_autotune
 *      基于辨识参数(R/L)用零极点对消法计算三环PID, 写入 ControlParam_t,
 *      按 ring_select 自动设 source=AUTOTUNE 并 reload。仅 IDLE 态可执行。
 * ==========================================================================*/
static jm_err_e app_pid_autotune(uint8_t ring_select, float cur_bw, float vel_bw, float pos_bw,
                                 uint8_t *out_fail_reason)
{
	if (out_fail_reason != NULL)
	{
		*out_fail_reason = 0;
	}

	/* 速率限制: 0xA0 PID_AUTOTUNE 复用 CALIB 间隔 (2s, 防并发启动) */
	if (!JM_RATE_CHECK(&s_last_tick_calib, JM_RATE_MIN_INTERVAL_CALIB_MS))
		return JM_ERR_RATE_LIMIT;

	/* 状态检查: 仅 IDLE 态允许(并发安全) */
	if (motor_loop_get()->sys.top_state != TOP_FSM_IDLE)
	{
		if (out_fail_reason != NULL)
		{
			*out_fail_reason = 2; /* 非 IDLE 态 */
		}
		return JM_ERR_STATE_DENY;
	}

	/* ring_mask 范围检查: bit0=电流 bit1=速度 bit2=位置, 0=空选无效, >0x07=越界 */
	if (ring_select == 0 || ring_select > 0x07)
	{
		if (out_fail_reason != NULL)
		{
			*out_fail_reason = 3; /* 参数无效 */
		}
		return JM_ERR_OUT_OF_RANGE;
	}

	/* 事务性计算并写入 ControlParam_t（按 ring_mask 仅计算所选环）*/
#if defined(USE_DEV_FLASH)
	motor_info_t *info = motor_info_storage_get();
#else
	motor_info_t *info = NULL; /* 未启用 Flash 存储: autotune 无法获取辨识参数, 返回参数无效 */
#endif
	int ret = motor_pid_autotune_apply(info, ring_select, cur_bw, vel_bw, pos_bw);
	if (ret != 0)
	{
		if (out_fail_reason != NULL)
		{
			*out_fail_reason = (ret == -2) ? 1 : 3; /* 1=辨识未就绪, 3=参数无效 */
		}
		return JM_ERR_STATE_DENY;
	}

	/* 按 ring_mask 自动设 source=AUTOTUNE (位掩码: bit0=电流 bit1=速度 bit2=位置) */
	if (ring_select & 0x01)
	{
		motor_pid_set_source(PID_RING_CURRENT, PID_SOURCE_AUTOTUNE);
	}
	if (ring_select & 0x02)
	{
		motor_pid_set_source(PID_RING_VELOCITY, PID_SOURCE_AUTOTUNE);
	}
	if (ring_select & 0x04)
	{
		motor_pid_set_source(PID_RING_POSITION, PID_SOURCE_AUTOTUNE);
	}

	/* 立即 reload 到运行期 */
	motor_pid_reload();

	/* 不自动保存 Flash, 由上位机显式发 0xEA 固化 */
	return JM_ERR_OK;
}

/* ============================================================================
 *  1c) PID 来源切换: CMD 0xA1  ->  pid_source_set
 *      独立设置某环参数来源(默认/Flash/理论估计/调试), 立即 reload。仅 IDLE 态可执行。
 *      source=3(DEBUG) 不持久化到 pid_source_mask, 重启自动消失。
 * ==========================================================================*/
static jm_err_e app_pid_source_set(uint8_t ring_select, uint8_t source)
{
	/* 状态检查: 仅 IDLE 态允许 */
	if (motor_loop_get()->sys.top_state != TOP_FSM_IDLE)
	{
		return JM_ERR_STATE_DENY;
	}

	/* 参数范围检查: ring_select 0=电流 1=速度 2=位置, source 0~3 */
	if (ring_select > 2 || source > (uint8_t)PID_SOURCE_DEBUG)
	{
		return JM_ERR_OUT_OF_RANGE;
	}

	/* 调用 PidManager API 设置 source */
	motor_pid_set_source((pid_ring_e)ring_select, (pid_source_e)source);

	/* 同步写入 Flash 持久化字段 pid_source_mask（RAM，由 0xEA 固化）
	 * DEBUG 不持久化: 重启自动回 DEFAULT/FLASH/AUTOTUNE */
	if (source != (uint8_t)PID_SOURCE_DEBUG)
	{
#if defined(USE_DEV_FLASH)
		motor_info_t *info = motor_info_storage_get();
		uint32_t mask = info->blocks.control.pid_source_mask;
		mask = pid_source_to_mask(mask, (pid_ring_e)ring_select, (pid_source_e)source);
		info->blocks.control.pid_source_mask = mask;
#endif
		/* 未启用 Flash: source 切换仅影响 RAM 中的运行期 profile, 不持久化 */
	}

	/* 立即 reload 生效 */
	motor_pid_reload();

	return JM_ERR_OK;
}

/* ----------------------------------------------------------------------------
 *  1d) PID 来源查询: CMD 0xA2  ->  pid_source_get
 *      返回三环当前 source 状态 (3字节: cur/vel/pos)
 * ==========================================================================*/
static jm_err_e app_pid_source_get(uint8_t *out_cur, uint8_t *out_vel, uint8_t *out_pos)
{
	if (out_cur == NULL || out_vel == NULL || out_pos == NULL)
	{
		return JM_ERR_OUT_OF_RANGE;
	}
	*out_cur = (uint8_t)motor_pid_get_source(PID_RING_CURRENT);
	*out_vel = (uint8_t)motor_pid_get_source(PID_RING_VELOCITY);
	*out_pos = (uint8_t)motor_pid_get_source(PID_RING_POSITION);
	return JM_ERR_OK;
}

/* ----------------------------------------------------------------------------
 *  1e) PID 参数实时写: CMD 0xA5  ->  pid_param_set
 *      仅 DEBUG source 下允许写, 直接写 s_motor_pid_profiles, ISR 下一拍生效。
 *      ring: 0=D轴 1=Q轴 2=速度 3=位置
 *      param_type: 1=kp 2=ki 3=kd 4=output_limit 5=integral_limit 6=output_filter_alpha 7=flags
 * ==========================================================================*/
static jm_err_e app_pid_param_set(uint8_t ring, uint8_t param_type, const uint8_t *value4)
{
	if (value4 == NULL || ring > 3 || param_type < 1 || param_type > 7)
	{
		return JM_ERR_OUT_OF_RANGE;
	}

	/* 检查对应环是否处于 DEBUG 模式 */
	pid_ring_e src_ring;
	if (ring <= 1)
		src_ring = PID_RING_CURRENT; /* D轴/Q轴 → 电流环 */
	else if (ring == 2)
		src_ring = PID_RING_VELOCITY;
	else
		src_ring = PID_RING_POSITION;

	if (motor_pid_get_source(src_ring) != PID_SOURCE_DEBUG)
	{
		return JM_ERR_STATE_DENY;
	}

	/* 直接写 s_motor_pid_profiles, ISR 下一拍生效 */
	if (motor_pid_profile_set_param(ring, param_type, value4) != 0)
	{
		return JM_ERR_OUT_OF_RANGE;
	}

	return JM_ERR_OK;
}

/* ----------------------------------------------------------------------------
 *  1f) PID 参数实时读: CMD 0xA6  ->  pid_param_get
 *      随时可读, 返回当前 profile 中的值 (4字节)。
 * ==========================================================================*/
static jm_err_e app_pid_param_get(uint8_t ring, uint8_t param_type, uint8_t *out_value4)
{
	if (out_value4 == NULL || ring > 3 || param_type < 1 || param_type > 7)
	{
		return JM_ERR_OUT_OF_RANGE;
	}

	if (motor_pid_profile_get_param(ring, param_type, out_value4) != 0)
	{
		return JM_ERR_OUT_OF_RANGE;
	}

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

	fb->pos = m->motion.position_rad;       /* 电机端多圈位置 θ_m rad(带符号,±∞) -> UI"电机位置" */
	fb->vel = m->motion.velocity_rad_s;     /* 电机端机械角速度 rad/s -> UI"电机速度" */
	fb->pos_ref = motor_loop_get()->sys.motor.ref.pos;
	fb->vel_ref = motor_loop_get()->sys.motor.ref.vel;
	fb->torque = m->power.torque_est;       /* 输出端力矩 Nm(估算) */
	fb->id = m->electrical.id_meas;         /* d轴电流 A */
	fb->iq = m->electrical.iq_meas;         /* q轴电流 A */
	fb->id_ref = motor_loop_get()->out.id_ref;
	fb->iq_ref = motor_loop_get()->out.iq_ref;
	fb->ia = m->electrical.ia;              /* A 相电流 A */
	fb->ib = m->electrical.ib;              /* B 相电流 A */
	fb->ic = m->electrical.ic;              /* C 相电流 A */
	fb->vbus = m->power.v_bus;              /* 母线电压 V */
	fb->ibus = m->power.i_bus;              /* 母线电流 A */
	fb->temp_fet = m->thermal.temp_fet;     /* 功率管温度 ℃ */
	fb->temp_motor = m->thermal.temp_motor; /* 电机温度 ℃ */
	fb->multiturn = m->motion.multiturn;    /* 多圈计数(整圈,带符号) -> UI"多圈计数" */
	fb->single = m->motion.single_turn_rad; /* 单圈机械角 rad [0,2π) -> UI"机械角度"(转°)/电角度计算源 */
	/* 返回锁存故障，避免瞬态条件消失后 FAULT 状态仍在但上位机显示无故障。 */
	fb->fault_mask = m->fault.fault_latched;
	fb->warn_mask = m->fault.warn_mask;     /* 警告掩码 */
	fb->top_fsm = (uint8_t)m->top_state;
	fb->run_state = (uint8_t)m->run_state;
	fb->ctrl_mode = (uint8_t)m->ctrl_mode;
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
	uint8_t type;    /* jm_param_type_e */
	uint8_t size;    /* 字段字节数 */
} param_desc_t;

/* 子结构字段 -> 全局偏移; size 由类型决定, 避免与 type 不一致 */
#define PT_SZ(t) ((t) == JM_PT_U8 || (t) == JM_PT_I8 ? 1 : (t) == JM_PT_U16 || (t) == JM_PT_I16 ? 2  \
	                                                   : (t) == JM_PT_STR                       ? 16 \
	                                                                                            : 4)

#define PARAM_ENT(grp, subtype, field, ptype)                                \
	{                                                                        \
		(uint16_t)(offsetof(motor_param_t, grp) + offsetof(subtype, field)), \
		(uint8_t)(ptype), (uint8_t)PT_SZ(ptype)}

/* 索引即 param_id(0~85), 顺序严格对齐 joint_motor_param_index.csv */
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
	PARAM_ENT(encoder_param, encoder_param_t, enc_offset, JM_PT_F32),
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
	/* 41~55 电流环 */
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
	PARAM_ENT(current_loop, current_loop_t, decouple_algo, JM_PT_U8),
	PARAM_ENT(current_loop, current_loop_t, bemf_ff_enable, JM_PT_U8),
	PARAM_ENT(current_loop, current_loop_t, deadtime_comp_enable, JM_PT_U8),
	/* 57~72 位置速度环 */
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
	/* 72~74 阻抗控制 */
	PARAM_ENT(impedance_ctrl, impedance_ctrl_t, impedance_kp, JM_PT_F32),
	PARAM_ENT(impedance_ctrl, impedance_ctrl_t, impedance_kd, JM_PT_F32),
	PARAM_ENT(impedance_ctrl, impedance_ctrl_t, iq_max, JM_PT_F32),
	/* 75~77 热模型 */
	PARAM_ENT(thermal_model, thermal_model_t, thermal_resistance, JM_PT_F32),
	PARAM_ENT(thermal_model, thermal_model_t, thermal_time_const, JM_PT_F32),
	PARAM_ENT(thermal_model, thermal_model_t, derating_temp_start, JM_PT_F32),
	/* 78~85 保护 */
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

static void app_param_snapshot(motor_param_t *shadow)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	memcpy(shadow, &usr.motor_param[M1], sizeof(*shadow));
	__set_PRIMASK(primask);
}

static void app_param_commit(const motor_param_t *shadow)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	memcpy(&usr.motor_param[M1], shadow, sizeof(*shadow));
	__set_PRIMASK(primask);
}

static jm_err_e app_param_write(uint16_t param_id, const uint8_t *value, uint8_t len)
{
	const param_desc_t *d;
	motor_param_t shadow;
	uint8_t *base;

	/* 速率限制: 0xE1 PARAM_WRITE 100ms 间隔 */
	if (!JM_RATE_CHECK(&s_last_tick_param_write, JM_RATE_MIN_INTERVAL_PARAM_WRITE_MS))
		return JM_ERR_RATE_LIMIT;
	/* 鉴权: 量产期启用时, 前 4 字节为令牌 */
	if (!JM_AUTH_CHECK(value, len))
		return JM_ERR_UNAUTHORIZED;
	JM_AUTH_SKIP(value, len); /* 鉴权通过, 跳过令牌 */

	if (param_id >= MOTOR_PARAM_PARAM_COUNT)
	{
		return JM_ERR_BAD_PARAM_ID;
	}
	d = &s_param_tbl[param_id];
	app_param_snapshot(&shadow);
	base = (uint8_t *)&shadow;
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
	if (motor_param_validate(&shadow) != 0)
		return JM_ERR_OUT_OF_RANGE;
	app_param_commit(&shadow);
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
	/* 速率限制: 0xE4 PARAM_SAVE 1s 间隔 (防 Flash 频繁擦写) */
	if (!JM_RATE_CHECK(&s_last_tick_param_save, JM_RATE_MIN_INTERVAL_PARAM_SAVE_MS))
		return JM_ERR_RATE_LIMIT;

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
		motor_param_init(&usr.motor_param[M1]);
		motor_profile_apply_param(&usr.motor_param[M1]);
		return JM_ERR_OK;
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
	motor_profile_apply_param(&def);
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
	motor_param_t shadow;
	uint8_t *base;
	uint16_t off = 0;
	uint16_t i;

	if (values == NULL && len != 0u)
		return JM_ERR_LENGTH;
	app_param_snapshot(&shadow);
	base = (uint8_t *)&shadow;

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
	if (off != len)
		return JM_ERR_LENGTH;
	if (motor_param_validate(&shadow) != 0)
		return JM_ERR_OUT_OF_RANGE;
	app_param_commit(&shadow);
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
	const char *name = (&usr.motor_param[M1])->motor_instance.motor_name;
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
static uint8_t s_tlm_enable = 0u;     /* 周期上报总开关: 0=停止, 1=启动 */
static uint16_t s_tlm_mask = 0xFFFFu; /* 默认订阅全部组 */
static uint16_t s_tlm_period_ms = 0u; /* 0 表示沿用绑定层默认周期 */

static jm_err_e app_set_telemetry(uint8_t enable, uint16_t mask, uint16_t period_ms)
{
	const uint16_t supported_mask = (uint16_t)(
		JM_TLM_POS_VEL | JM_TLM_DQ | JM_TLM_PHASE | JM_TLM_BUS |
		JM_TLM_TEMP | JM_TLM_MULTITURN | JM_TLM_TORQUE | JM_TLM_FAULT |
		JM_TLM_STATE | JM_TLM_DEBUG | JM_TLM_CURRENT_TARGET |
		JM_TLM_MOTION_TARGET);
	if ((mask & (uint16_t)~supported_mask) != 0u)
	{
		return JM_ERR_OUT_OF_RANGE;
	}
	if (enable && mask == 0u)
	{
		mask = supported_mask;
	}
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
 *     CAN ID 原子写入 motor_info Flash, 由 CAN 绑定层在下次启动时加载。
 * ==========================================================================*/
static uint8_t s_can_baud_code = 0u; /* 0=1M(默认) 1=500K 2=250K 3=125K */

static jm_err_e app_set_can_id(uint8_t new_id)
{
	if (motor_loop_get()->sys.top_state != TOP_FSM_IDLE)
		return JM_ERR_STATE_DENY;
	/* Flash 写命令使用 1s 限流, 防止反复改地址损耗存储寿命。 */
	if (!JM_RATE_CHECK(&s_last_tick_motor_info_s, JM_RATE_MIN_INTERVAL_MOTOR_INFO_S_MS))
		return JM_ERR_RATE_LIMIT;
	/* 鉴权: 由 dispatch 层在 payload 前缀校验 (若启用), 本层不处理 */

#if defined(USE_DEV_FLASH)
	{
		motor_info_t *cfg = motor_info_storage_get();
		uint32_t old_id;
		motor_info_storage_status_t rc;

		if (cfg == NULL)
			return JM_ERR_FLASH;
		old_id = cfg->blocks.device.can_id;
		if (motor_info_write_u32(cfg, MOTOR_INFO_PID_CAN_ID, new_id) != 0)
			return JM_ERR_OUT_OF_RANGE;

		rc = jm_app_motor_info_storage_save(cfg);
		if (rc == MOTOR_INFO_STORAGE_OK)
			return JM_ERR_OK;

		/* 保存失败时恢复运行期配置; CAN 协议和硬件过滤器始终保持旧地址。 */
		(void)motor_info_write_u32(cfg, MOTOR_INFO_PID_CAN_ID, old_id);
		if (rc == MOTOR_INFO_STORAGE_ERR_FLASH_WRITE)
			return JM_ERR_FLASH_WRITE;
		if (rc == MOTOR_INFO_STORAGE_ERR_FLASH_VERIFY)
			return JM_ERR_FLASH_VERIFY;
		return (rc > 0) ? JM_ERR_OUT_OF_RANGE : JM_ERR_FLASH;
	}
#else
	(void)new_id;
	return JM_ERR_UNSUPPORTED;
#endif
}

static jm_err_e app_identify_can_device(uint8_t duration_100ms)
{
#if defined(USE_DEV_LED)
	led_manager_identify((uint32_t)duration_100ms * 100u);
	return JM_ERR_OK;
#else
	(void)duration_100ms;
	return JM_ERR_UNSUPPORTED;
#endif
}

static jm_err_e app_set_baudrate(uint8_t baud_code)
{
	/* 速率限制: 0xF1 SET_BAUDRATE 复用 PARAM_WRITE 间隔 */
	if (!JM_RATE_CHECK(&s_last_tick_param_write, JM_RATE_MIN_INTERVAL_PARAM_WRITE_MS))
		return JM_ERR_RATE_LIMIT;

	s_can_baud_code = baud_code;
	return JM_ERR_OK;
}

uint8_t jm_app_can_baudrate(void)
{
	return s_can_baud_code;
}

/* 0xF3 SET_FD_MODE: 切换 CAN FD 运行期模式(纯软件操作, 不重新初始化 FDCAN 外设)。
 * - CAN 模式(USE_DEV_COMMUN_CAN): 调用 jm_proto_can_set_fd_mode, 同步 dev_commun_can.use_fd_runtime
 * - UART 模式或未启用 CAN: 返回 cap=0
 * ACK 时序: jm_proto_can_set_fd_mode 在 c->use_fd_runtime 写入后立即返回,
 *           但 ACK 由 can_emit_payload 用旧模式发出(因 ACK 在 dispatch 中设置 reply 后才发送,
 *           而 jm_can_tx 读取的是已切换后的 use_fd_runtime)。
 *           为保证"ACK 用旧模式发出", 这里在调用 set_fd_mode 前先记录 ack_enable,
 *           然后延迟到 ACK 发送完成后再切换 —— 但当前架构 ACK 发送在 dispatch 返回后,
 *           无法在 ACK 发送后回调。简化: 双方都遵循"收到 ACK 后切换自身模式",
 *           切换瞬间收发模式短暂不一致由 FD 控制器硬件兼容性兜底(FD 控制器可收经典帧)。*/
static jm_err_e app_set_fd_mode(uint8_t enable, uint8_t *out_ack_enable, uint8_t *out_cap)
{
	if (out_ack_enable == NULL || out_cap == NULL)
	{
		return JM_ERR_BAD_PARAM_ID;
	}

#if defined(USE_DEV_COMMUN_CAN)
	{
		uint8_t cap = jm_proto_can_set_fd_mode(&dev_commun_can.jm, enable);
		/* 同步设备层镜像(供 jm_can_tx/on_rx_msg 读取) */
		dev_commun_can.use_fd_runtime = (enable && cap) ? 1u : 0u;
		*out_cap = cap;
		*out_ack_enable = (enable && cap) ? 1u : 0u;
		return JM_ERR_OK;
	}
#else
	/* UART 模式: 无 CAN FD 能力 */
	*out_cap = 0u;
	*out_ack_enable = 0u;
	return JM_ERR_OK;
#endif
}

/* ============================================================================
 *  8) 电机配置(motor_info)读写: CMD 0xE6~0xEB
 *     与 0xE0-0xE5 的运行时参数(motor_param_t)独立, 面向 Flash/EEPROM 持久化
 *     的硬件配置/校准数据。帧内 value 固定 4 字节, 由 motor_info_dispatch
 *     按字段类型(u8/i8/u16/i16/u32/i32/f32)自动转换。
 *     全局 g_motor_info 实例由 motor_info_storage 模块封装持有:
 *       - motor_info_storage_init() 上电自动加载(默认+Flash+profile)
 *       - motor_info_storage_get() 返回已初始化的句柄
 * ==========================================================================*/

/* motor_info 持久化: 弱实现仅做范围校验, 不落 Flash。
 * 接入 Flash 驱动后在驱动层提供同名强符号覆盖(类比 jm_app_param_storage_save)。
 * 返回值类型 motor_info_storage_status_t：0=成功, >0=越界 param_id, <0=系统错误。
 * 未启用 USE_DEV_FLASH 时本弱符号不编译: 0xE6-0xEB 整组命令在 ops 表中置 NULL,
 * jm_proto_dispatch 已对 ops->motor_info_xxx 做 NULL 检查并返回 NACK(UNSUPPORT)。*/
#if defined(USE_DEV_FLASH)
#if defined(__GNUC__) || defined(__clang__)
__attribute__((weak))
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
__weak
#endif
motor_info_storage_status_t jm_app_motor_info_storage_save(const motor_info_t *cfg)
{
	return (motor_info_storage_status_t)motor_info_validate(cfg); /* 0=全部通过, 否则首个越界 param_id(>0) */
}
#endif /* USE_DEV_FLASH */

/* dispatch 返回码 -> jm_err_e */
static jm_err_e mi_dispatch_to_err(int rc)
{
	switch (rc)
	{
		case MOTOR_INFO_DISPATCH_OK:
			return JM_ERR_OK;
		case MOTOR_INFO_DISPATCH_E_BAD_ID:
			return JM_ERR_BAD_PARAM_ID;
		case MOTOR_INFO_DISPATCH_E_BOUNDS:
			return JM_ERR_OUT_OF_RANGE;
		case MOTOR_INFO_DISPATCH_E_RO:
			return JM_ERR_READ_ONLY;
		default:
			return JM_ERR_FLASH;
	}
}

/* ---- 0xE6-0xEC motor_info 命令组: 仅在启用 USE_DEV_FLASH 时提供实现,
 *      未启用时本组函数不编译, s_app_ops 表对应字段置 NULL,
 *      jm_proto_dispatch 对 ops->motor_info_xxx 已做 NULL 检查并返回 NACK。---- */
#if defined(USE_DEV_FLASH)

/* ---- 0xE6 读单个电机配置 ---- */
static jm_err_e app_motor_info_read(uint16_t param_id, uint8_t *value4,
                                    uint8_t *out_type, uint8_t *out_len)
{
	int rc = motor_info_dispatch_read(param_id, motor_info_storage_get(),
	                                  value4, out_type, out_len);
	return mi_dispatch_to_err(rc);
}

/* ---- 0xE7 写单个电机配置(RAM, 需 0xEA 固化) ---- */
static jm_err_e app_motor_info_write(uint16_t param_id, const uint8_t *value4, uint8_t len)
{
	/* 速率限制: 0xE7 MOTOR_INFO_WRITE 100ms 间隔 */
	if (!JM_RATE_CHECK(&s_last_tick_motor_info_w, JM_RATE_MIN_INTERVAL_MOTOR_INFO_W_MS))
		return JM_ERR_RATE_LIMIT;

	int rc = motor_info_dispatch_write(param_id, motor_info_storage_get(), value4, len);
	return mi_dispatch_to_err(rc);
}

/* ---- 0xEA 把 motor_info 整块写入 Flash ---- */
static jm_err_e app_motor_info_save(void)
{
	/* 速率限制: 0xEA MOTOR_INFO_SAVE 1s 间隔 (防 Flash 频繁擦写) */
	if (!JM_RATE_CHECK(&s_last_tick_motor_info_s, JM_RATE_MIN_INTERVAL_MOTOR_INFO_S_MS))
		return JM_ERR_RATE_LIMIT;

	motor_info_storage_status_t rc = jm_app_motor_info_storage_save(motor_info_storage_get());
	if (rc == MOTOR_INFO_STORAGE_OK)
	{
		return JM_ERR_OK;
	}
	/* 详细错误码映射:
	 *   rc > 0 = 越界 param_id -> OUT_OF_RANGE
	 *   rc = ERR_FLASH_WRITE  -> JM_ERR_FLASH_WRITE (0x11, 擦写失败)
	 *   rc = ERR_FLASH_VERIFY -> JM_ERR_FLASH_VERIFY (0x12, 回读校验失败)
	 *   rc = 其他 < 0         -> JM_ERR_FLASH (0x08, 通用 Flash 错误) */
	if (rc == MOTOR_INFO_STORAGE_ERR_FLASH_WRITE)
		return JM_ERR_FLASH_WRITE;
	if (rc == MOTOR_INFO_STORAGE_ERR_FLASH_VERIFY)
		return JM_ERR_FLASH_VERIFY;
	return (rc > 0) ? JM_ERR_OUT_OF_RANGE : JM_ERR_FLASH;
}

/* ---- 0xE8 批量读(块内连续ID有效, 跨块间隔返回 BAD_PARAM_ID) ---- */
static jm_err_e app_motor_info_read_bulk(uint16_t start_id, uint16_t count,
                                         uint8_t *out, uint16_t *out_len)
{
	uint16_t i, n = 0;
	motor_info_t *cfg = motor_info_storage_get();

	if (out == NULL || out_len == NULL)
	{
		return JM_ERR_BAD_PARAM_ID;
	}
	/* 应答头: start_id(u16) + count(u8), 预留最多 count*4B */
	out[0] = (uint8_t)(start_id & 0xFF);
	out[1] = (uint8_t)((start_id >> 8) & 0xFF);
	/* count 占位, 末尾回填实际成功数 */
	n = 3;
	for (i = 0; i < count; i++)
	{
		uint8_t v4[4], tcode = 0, vlen = 0;
		uint16_t pid = (uint16_t)(start_id + i);
		int rc = motor_info_dispatch_read(pid, cfg, v4, &tcode, &vlen);
		if (rc != MOTOR_INFO_DISPATCH_OK)
		{
			/* 遇到无效ID(块间隔/越界)即停止, 已读部分仍有效 */
			break;
		}
		/* 超单帧容量(JM_PAYLOAD_MAX)则截断 */
		if ((uint16_t)(n + 4u) > JM_PAYLOAD_MAX)
		{
			break;
		}
		out[n++] = v4[0];
		out[n++] = v4[1];
		out[n++] = v4[2];
		out[n++] = v4[3];
	}
	out[2] = (uint8_t)i; /* 实际成功读取数 */
	*out_len = n;
	return JM_ERR_OK;
}

/* ---- 0xE9 批量写(块内连续ID有效) ---- */
static jm_err_e app_motor_info_write_bulk(uint16_t start_id, uint16_t count,
                                          const uint8_t *values, uint16_t len)
{
	uint16_t i;
	motor_info_t *cfg = motor_info_storage_get();

	if (values == NULL || len < (uint16_t)(count * 4u))
	{
		return JM_ERR_LENGTH;
	}
	for (i = 0; i < count; i++)
	{
		uint16_t pid = (uint16_t)(start_id + i);
		int rc = motor_info_dispatch_write(pid, cfg, &values[i * 4u], 4u);
		if (rc != MOTOR_INFO_DISPATCH_OK)
		{
			/* 首个失败即终止, 返回对应错误码(已写部分保留) */
			return mi_dispatch_to_err(rc);
		}
	}
	return JM_ERR_OK;
}

/* ---- 0xEB 恢复默认(param_id=0xFFFF 全部; 单参从默认实例回写) ---- */
static jm_err_e app_motor_info_reset(uint16_t param_id)
{
	motor_info_t *cfg = motor_info_storage_get();

	if (param_id == 0xFFFFu)
	{
		motor_info_init(cfg);
		motor_profile_apply_info(cfg);
		return JM_ERR_OK;
	}
	/* 单参恢复: 从默认实例读出该参数值, 再写入当前实例 */
	{
		motor_info_t def;
		uint8_t v4[4], tcode = 0, vlen = 0;
		int rc;
		if (motor_info_init(&def) != 0)
		{
			return JM_ERR_FLASH;
		}
		motor_profile_apply_info(&def);
		rc = motor_info_dispatch_read(param_id, &def, v4, &tcode, &vlen);
		if (rc != MOTOR_INFO_DISPATCH_OK)
		{
			return mi_dispatch_to_err(rc);
		}
		rc = motor_info_dispatch_write(param_id, cfg, v4, vlen);
		return mi_dispatch_to_err(rc);
	}
}

/* ---- 0xEC 清除标定状态以便重新标定（保留电机本体参数）----
 * 清除 is_calibrated + 编码器字段（enc_offset/elec_angle_bias/enc_direction），
 * 保留电气字段和限幅字段。仅清 RAM，不自动落盘，上位机需随后发 0xEA 固化。*/
static jm_err_e app_motor_info_recalib_reset(void)
{
	int rc = motor_info_calib_reset_for_recalibration();
	if (rc != 0)
		return JM_ERR_FLASH;
	return JM_ERR_OK;
}

#endif /* USE_DEV_FLASH */

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
	.set_fd_mode = app_set_fd_mode,
	.identify_can_device = app_identify_can_device,
	/* 电机配置(motor_info) 0xE6-0xEB (未启用 USE_DEV_FLASH 时置 NULL, 命令返回 NACK) */
#if defined(USE_DEV_FLASH)
	.motor_info_read = app_motor_info_read,
	.motor_info_write = app_motor_info_write,
	.motor_info_save = app_motor_info_save,
	.motor_info_read_bulk = app_motor_info_read_bulk,
	.motor_info_write_bulk = app_motor_info_write_bulk,
	.motor_info_reset = app_motor_info_reset,
	.motor_info_recalib_reset = app_motor_info_recalib_reset,
#endif
	/* PID 管理 0xA0~0xA6 */
	.pid_autotune = app_pid_autotune,
	.pid_source_set = app_pid_source_set,
	.pid_source_get = app_pid_source_get,
	.pid_param_set = app_pid_param_set,
	.pid_param_get = app_pid_param_get,
};

const jm_proto_ops_t *jm_app_ops_get(void)
{
	return &s_app_ops;
}
