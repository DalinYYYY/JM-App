/**
 * @file        fault_manager.h
 * @brief       故障管理器：异常检测仲裁、分级处理、历史记录
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-09-03
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者  | 修改内容   |
 * |------------|------|-------|------------|
 * | 2026-09-03 | 1.0  | Dalin | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 *
 * @details 故障管理唯一事实源：码表/记录/仲裁/级别动作/历史全部收敛于本模块，
 *          system_state 仅保留状态机职责(收掩码切 FAULT 态)。
 *
 *          分级处理策略(动作由 fault_def 码表逐项定义, 级别决定恢复语义):
 *            故障级 -> 立即停机(FAULT 态), 人工清障恢复
 *            异常级 -> 按动作降功率或停机, 故障消除后手动清除
 *            警告级 -> 仅记录, 条件消失自动清除
 *
 *          调用上下文:
 *            fault_mgr_poll_fast  - 10kHz ISR (motor_control_loop 内, 经 fault_check)
 *            fault_mgr_poll_slow  - 5ms 线程 (thread_period)
 *            fault_mgr_report     - 任意上下文(短临界区保护)
 *            其余查询接口        - 任意上下文(只读)
 */

#ifndef __FAULT_MANAGER_H__
#define __FAULT_MANAGER_H__

#include <stdint.h>
#include <stdbool.h>
#include "fault_def.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* 历史环形缓冲深度(系统异常规范: 100 条循环覆盖) */
#define FAULT_HISTORY_DEPTH 100u

/* 清除标志(fault_mgr_clear) */
#define FAULT_CLEAR_LATCHED (1u << 0) /* 清锁存故障(默认行为, 兼容旧 0xB0) */
#define FAULT_CLEAR_HISTORY (1u << 1) /* 清历史记录 */

/*****************************************************************************
 * @brief   异常检测项编译期启用开关(1=启用 0=禁用)
 * @note    当前仅保留: 受 protect_enable 总使能控制的电气类快速检测
 *          (0x3102/0x2102/0x2103/0x4106/0x3101)与慢速母线过流/堵转
 *          (0x2104/0x2203/0x4202); 其余检测项暂不启动, 需要时置 1 恢复
 *****************************************************************************/
#define FAULT_DET_EN_ALGO_DIVERGE 0u /* 0x810A 算法发散(NaN/Inf) */
#define FAULT_DET_EN_VBUS_SAMPLE  0u /* 0x2105 VBUS 采样异常 */
#define FAULT_DET_EN_VBUS_MID     0u /* 0x2201/0x2202 过压/欠压中度 */
#define FAULT_DET_EN_ENC_HEALTH   0u /* 0x5101/0x5102/0x5105 编码器位置丢失/消磁/通讯中断 */
#define FAULT_DET_EN_ENC_JUMP     0u /* 0x5104 位置跳变 */
#define FAULT_DET_EN_FOLLOW_ERR   0u /* 0x8108 跟随误差 */
#define FAULT_DET_EN_SOFT_LIMIT   0u /* 0x1201/0x1301 软限位触发/接近警告 */
#define FAULT_DET_EN_THERMAL      0u /* 0x3205/0x4205 NTC采样 + 0x32xx/0x41xx/0x43xx 温度分级(慢速) */

/* 快检测错峰分频: fault_mgr_poll_fast 全量电气检测每 N 个电流环拍执行一次
 * (N=FAULT_DET_FAST_DIV, 1=不分频每拍执行; NaN 发散检测不受分频, 见
 * fault_detect_diverge 每拍)。消抖/武装阈值按采样周期(控制周期×N)换算,
 * 保护响应时间不变。
 * 完整错峰调度(motor_loop_config.h): 速度拍0/VEL_DIV、位置拍4、故障拍2、
 * 遥测拍7; PLL 速度解算严格只发生在速度拍(间隔均匀), 经判别实验确认故障
 * 错峰与控制环路零耦合 */
#define FAULT_DET_FAST_DIV 10u

	/*****************************************************************************
 * @brief   故障管理运行配置(阈值/策略)
 * @note    默认值编译期固化, 上电后由 motor_info FaultParam 块加载覆盖
 *          (任务7接入; 未配置时保持默认, 兼容旧参数区)。
 *          enable_mask 按级别粒度控制(细粒度逐故障开关二期评估);
 *          action_policy: 0=按码表动作 1=全部按故障级停机(保守) 2=全部仅记录(调试)
 *****************************************************************************/
	typedef struct
	{
		/* 中度电压阈值(异常级降功率) */
		float ov_mid_v; /* 中度过压阈值 V */
		float uv_mid_v; /* 中度欠压阈值 V */
		/* 母线电流 */
		float ibus_over_A;      /* 母线过流阈值 A (故障级) */
		float ibus_cont_A;      /* 持续过流阈值 A (异常级) */
		float ibus_cont_time_s; /* 持续过流判定时长 s */
		/* 温度阈值 */
		float temp_fet_over_d;    /* 驱动器过温 ℃ (异常级降功率) */
		float temp_motor_over_d;  /* 电机过温熔断 ℃ (故障级) */
		float temp_motor_hot_d;   /* 绕组过热 ℃ (异常级降功率) */
		float temp_under_d;       /* 低温阈值 ℃ (DENY 禁止运行) */
		float temp_warn_offset_d; /* 预警偏移 ℃ (过温阈值-偏移=预警线) */
		/* 堵转判据 */
		float stall_vel;       /* 堵转速度阈值 rad/s */
		float stall_current_A; /* 堵转电流阈值 A */
		float stall_time_s;    /* 堵转判定时长 s */
		/* 编码器 */
		float pos_jump_deg;      /* 单周期位置跳变阈值 ° */
		uint32_t enc_err_frames; /* 连续坏帧判定帧数 */
		/* 跟随误差 */
		float follow_err_rad; /* 位置跟随误差阈值 rad(输出端) */
		/* 软限位 */
		float soft_limit_min_rad;  /* 软限位下限 rad */
		float soft_limit_max_rad;  /* 软限位上限 rad */
		float soft_limit_warn_deg; /* 接近软限位警告距离 ° */
		/* 降功率系数 */
		float derate_mid;    /* 中度降功率系数(默认0.7) */
		float derate_severe; /* 重度降功率系数(默认0.5) */
		/* 采样有效性 */
		uint32_t vbus_invalid_ms; /* VBUS 采样无效判定时长 ms */
		/* 策略 */
		uint64_t enable_mask[3]; /* [0]故障级 [1]异常级 [2]警告级 使能掩码(级别总开关: 0xFFFFFFFFFFFFFFFF 全使能, 非0=启用) */
		uint8_t action_policy[3]; /* [级别] 0=按码表 1=全部停机 2=仅记录 */
	} fault_cfg_t;

	/*****************************************************************************
 * @brief   故障活动记录(每码一条, index=fault_code_to_index)
 *****************************************************************************/
	typedef struct
	{
		uint32_t first_ms; /* 首次发生时间戳 ms */
		uint32_t last_ms;  /* 最近发生时间戳 ms */
		uint32_t count;    /* 累计发生次数 */
		float value;       /* 触发时刻物理量(最新) */
		uint8_t status;    /* fault_status_e */
	} fault_rec_t;

	/*****************************************************************************
 * @brief   历史记录条目(环形缓冲, 100 条循环覆盖)
 *****************************************************************************/
	typedef struct
	{
		uint16_t code;     /* 故障码 */
		uint16_t _rsv;     /* 对齐 */
		uint32_t first_ms; /* 首次发生时间戳 ms */
		uint32_t last_ms;  /* 最近发生时间戳 ms */
		uint32_t count;    /* 发生次数 */
		float value;       /* 触发值 */
	} fault_hist_t;

	/*****************************************************************************
 * @brief   故障管理器上下文
 *****************************************************************************/
	typedef struct fault_mgr_s
	{
		fault_rec_t rec[FAULT_CODE_COUNT];      /* 活动记录表(码表序) */
		fault_hist_t hist[FAULT_HISTORY_DEPTH]; /* 历史环形缓冲 */
		uint16_t hist_head;                     /* 环形写指针(下一条写入位置) */
		uint16_t hist_count;                    /* 有效历史条数 */
		fault_cfg_t cfg;                        /* 运行配置(默认值+参数块覆盖) */

		/* 仲裁结果(每拍重算) */
		uint16_t top_fault;    /* 最高优先级活动故障码(0=无) */
		uint16_t active_cnt;   /* 活动故障总数 */
		uint32_t level_active; /* bit0/1/2 = 故障级/异常级/警告级有活动 */
		float derate;          /* 当前降功率系数(1.0=正常) */
		uint8_t clamp_active;  /* 软限位钳制激活(CLAMP_POS 动作) */
		uint8_t deny_enable;   /* 禁止使能标志(DENY 动作: 未标定/低温) */

		/* 兼容旧状态机字段(system_state 派生映射源) */
		uint32_t compat_mask;    /* 旧 fault_mask 语义(低13位=旧6故障映射) */
		uint32_t warn_mask;      /* warn_mask 语义(异常级bit0-15/警告级bit16-31, index&15) */
		uint32_t fault_latched;  /* 停机类故障锁存(需 CLEAR_FAULT) */
		uint16_t fault_count;    /* 累计故障次数 */
		uint8_t last_fault_code; /* 最近一次故障位编号(旧语义) */

		/* 运行时辅助 */
		uint32_t uptime_ms;         /* 时间戳缓存(report 时刷新) */
		struct system_state_s *sys; /* 绑定的状态机(动作执行) */
		uint32_t dirty;             /* 记录变化标志(eval快速路径: 置位须重算, 重算后清零) */

		/* 检测器辅助状态(自 system_state 迁移/新增, 仅 detect 文件访问) */
		uint32_t speed_guard_cycles;     /* 启动超速保护延迟武装剩余拍数 */
		uint32_t vbus_guard_cycles;      /* VBUS 采样异常检测上电武装剩余拍数(fb 数据链路就绪前防误锁存) */
		uint32_t gate_fault_cycles;      /* nFAULT 连续拉低计数(消抖) */
		uint32_t gate_fault_confirm_cyc; /* nFAULT 确认阈值拍数 */
		uint32_t vbus_invalid_cycles;    /* VBUS 采样无效累计拍数 */
		uint32_t ibus_cont_cycles;       /* 持续过流累计拍数(slow, 5ms/拍) */
            uint32_t stall_cycles;           /* 堵转累计拍数(slow, 5ms/拍) */
		uint32_t over_current_cycles;    /* 峰值过流累计拍数(消抖) */
		uint32_t over_current_confirm_cyc; /* 峰值过流确认阈值拍数 */
		uint32_t over_speed_cycles;      /* 超速累计拍数(消抖) */
		uint32_t over_speed_confirm_cyc; /* 超速确认阈值拍数 */
		uint8_t ntc_fet_bad_cycles;      /* 驱动器NTC异常累计拍数(slow) */
		uint8_t ntc_motor_bad_cycles;    /* 电机NTC异常累计拍数(slow) */
		float enc_prev_deg;              /* 编码器上拍机械角度 °(跳变检测) */
		uint8_t enc_prev_valid;          /* 编码器上拍角度有效标志 */
	} fault_mgr_t;

	/*****************************************************************************
 * @brief   初始化故障管理器
 * @note    复位全部记录/历史, cfg 置默认值; 检查 RCC 复位标志补录看门狗故障
 *          (0x8101, IWDG 复位后 RAM 历史已清空, 唯一留痕时机)。
 *          须在 usr 数据初始化后、状态机启动前调用。
 *****************************************************************************/
	void fault_mgr_init(void);

	/*****************************************************************************
 * @brief   绑定系统状态机(动作执行需切换 FAULT 态)
 * @param   sys 系统状态机指针(非NULL)
 * @note    由 system_state_init 调用; fault_manager.c 单向依赖 system_state.h
 *****************************************************************************/
	void fault_mgr_attach(struct system_state_s *sys);

	/*****************************************************************************
	 * @brief   快速故障检测与仲裁(错峰分频, 1kHz @10kHz 电流环)
	 * @note    由 fault_check 委托调用: 执行电气类快检测(过流/电压/超速/
	 *          nFAULT消抖/编码器/跟随误差/软限位), 随后重算仲裁结果并执行
	 *          级别动作(停机类故障直接切 FAULT 态)。
	 *          调度: 每 FAULT_DET_FAST_DIV 拍一次(motor_loop 错峰拍), 消抖
	 *          阈值已按采样周期换算, 保护响应时间不变;
	 *          NaN 发散检测不走本接口(fault_detect_diverge 每拍)。
	 *          单次执行预算 < 10µs(检测14项标量比较+109项仲裁遍历空闲快速路径)。
	 *****************************************************************************/
	void fault_mgr_poll_fast(void);

	/*****************************************************************************
 * @brief   慢速故障检测与仲裁(5ms 线程)
 * @note    由 thread_period 调用: 温度分级/采样有效性/持续过流计时/堵转/
 *          编码器统计。检测项自带时间常数, 拍周期取 THREAD_DELAY_PERIOD。
 *****************************************************************************/
	void fault_mgr_poll_slow(void);

	/*****************************************************************************
 * @brief   外部事件上报(任意上下文)
 * @param   code 故障码(fault_code_e)
 * @param   value 触发物理量(无则0)
 * @note    急停(0x1101)/参数越界(0x8105)/EEPROM错误(0x8205)等非周期检测
 *          事件由事发点调用。短临界区保护, ISR/线程双上下文安全。
 *****************************************************************************/
	void fault_mgr_report(uint16_t code, float value);

	/*****************************************************************************
 * @brief   清除故障/历史
 * @param   flags FAULT_CLEAR_LATCHED|FAULT_CLEAR_HISTORY
 * @note    清锁存: 停机类故障条件已消失的记录置 CLEARED, 仍活动的拒绝
 *          (旧 0xB0 语义: fault_code==0 才允许离开 FAULT 态)。
 *          警告级记录条件消失后自动清除, 无需手动。
 *****************************************************************************/
	void fault_mgr_clear(uint8_t flags);

	/* ---- 查询接口(只读, 任意上下文) ---- */
	float fault_mgr_get_derate(void);       /* 降功率系数 1.0/0.7/0.5 */
	uint16_t fault_mgr_get_top_fault(void); /* 最高优先级活动故障码(0=无) */
	uint16_t fault_mgr_active_count(void);  /* 活动故障总数 */
	uint32_t fault_mgr_level_active(void);  /* bit0/1/2=故障/异常/警告级有活动 */
	uint32_t fault_mgr_get_warn_mask(void); /* 警告掩码(异常级bit0-15/警告级bit16-31) */
	uint8_t fault_mgr_deny_enable(void);    /* 1=禁止使能(DENY: 未标定/低温) */
	fault_cfg_t *fault_mgr_get_cfg(void);   /* 配置指针(FaultParam 加载用) */
	uint8_t fault_mgr_is_enabled(uint16_t code); /* 检查故障是否使能(冷路径, 二分查索引; 热路径用 is_enabled_idx) */
	uint8_t fault_mgr_is_enabled_idx(uint8_t idx); /* O(1) 索引版(热路径, idx=FAULT_IDX_*) */
	void fault_mgr_set_enable_mask(const uint64_t mask[3]); /* 原子更新三级使能并清理已禁用级别活动故障 */

	/*****************************************************************************
 * @brief   软限位钳制(CLAMP_POS 动作)
 * @param   pos [in/out] 位置参考 rad
 * @note    无软限位故障时原值通过; 触发后钳到边界(允许反向运动)。
 *          由 cascade_control 位置入环点调用。
 *****************************************************************************/
	void fault_mgr_clamp_pos(float *pos);

	/*****************************************************************************
 * @brief   活动故障记录查询(优先级序)
 * @param   idx 序号(0=最高优先级, < fault_mgr_active_count)
 * @retval  记录指针(code 需经 fault_code_to_index 反查, 此处直接给出);
 *          越界返回 NULL
 *****************************************************************************/
	const fault_rec_t *fault_mgr_get_active(uint8_t idx, uint16_t *code);

	/*****************************************************************************
 * @brief   历史记录查询(时间倒序)
 * @param   idx 序号(0=最新, < fault_mgr_history_count)
 * @retval  历史条目指针; 越界返回 NULL
 *****************************************************************************/
	const fault_hist_t *fault_mgr_get_history(uint8_t idx);
	uint16_t fault_mgr_history_count(void);

	/*****************************************************************************
 * @brief   同步兼容字段到旧状态机结构(fault_check 每拍调用)
 * @note    sys->fault_code/fault_latched/fault_count/last_fault_code 从
 *          fault_mgr 派生, 保持旧 motor_loop_sync_state/协议 0xCA 遥测路径不变。
 *****************************************************************************/
	void fault_mgr_sync_compat(struct system_state_s *sys);

	/* ---- 检测实现(fault_detect_fast/slow.c, 由 poll 调用) ---- */
	void fault_detect_fast(fault_mgr_t *fm);
	void fault_detect_slow(fault_mgr_t *fm);

	/* ---- 检测器记录接口(仅 fault_detect_*.c 使用) ----
	 * 条件成立调用 set(首次触发记录+历史, 重复触发计数);
	 * 条件消失调用 clear(警告级/非停机异常级自动解除, 停机类锁存)。
	 * _idx 版为 O(1) 热路径(10kHz 快检测): idx=FAULT_IDX_* 编译期绑定,
	 *     免二分查找; code 版供慢速检测/外部上报等冷路径。 */
	void fault_mgr_internal_set(uint16_t code, float value);
	void fault_mgr_internal_clear(uint16_t code);
	void fault_mgr_internal_set_idx(uint8_t idx, float value);
	void fault_mgr_internal_clear_idx(uint8_t idx);

	/* 管理器单例(协议层 0xAA 直接读仲裁结果, 只读语义) */
	extern fault_mgr_t g_fault_mgr;

	/* 快速检测消抖阈值按采样周期换算(attach 时由 控制周期×FAULT_DET_FAST_DIV 计算) */
	void fault_detect_fast_init_cycles(fault_mgr_t *fm, float sample_dt);

	/* NaN 算法发散检测(每拍, 不受 FAULT_DET_FAST_DIV 分频; 编译开关 FAULT_DET_EN_ALGO_DIVERGE) */
	void fault_detect_diverge(fault_mgr_t *fm);

#ifdef __cplusplus
}
#endif
#endif /* __FAULT_MANAGER_H__ */
