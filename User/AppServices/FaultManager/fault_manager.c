/**
 * @file        fault_manager.c
 * @brief       故障管理器实现：记录/仲裁/级别动作/历史/兼容映射
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
 * @details 数据流: detect(fast/slow) -> report(记录+历史) -> eval(仲裁+动作)。
 *          动作执行依赖 attach 的 system_state(切 FAULT 态), 与原 fault_check
 *          在 ISR 内调 top_fsm_switch 的上下文一致, 无新增约束。
 */

#include "fault_manager.h"
#include "system_state.h"
#include "runtime_param.h"
#include "main.h" /* RCC 复位标志读取 */
#include <string.h>

fault_mgr_t g_fault_mgr; /* 单例(工程无多电机故障管理需求) */

/* 配置默认值(与 protect_comm 旧默认对齐; FaultParam 块加载后覆盖) */
static const fault_cfg_t s_cfg_default = {
	.ov_mid_v = 56.0f,
	.uv_mid_v = 17.0f,
	.ibus_over_A = 30.0f,
	.ibus_cont_A = 15.0f,
	.ibus_cont_time_s = 10.0f,
	.temp_fet_over_d = 85.0f,
	.temp_motor_over_d = 120.0f,
	.temp_motor_hot_d = 90.0f,
	.temp_under_d = -20.0f,
	.temp_warn_offset_d = 10.0f,
	.stall_vel = 0.5f,
	.stall_current_A = 15.0f,
	.stall_time_s = 2.0f,
	.follow_err_rad = 0.5f,
	.pos_jump_deg = 10.0f,
	.enc_err_frames = 10u,
	.soft_limit_min_rad = -100.0f,
	.soft_limit_max_rad = 100.0f,
	.soft_limit_warn_deg = 5.0f,
	.derate_mid = 0.7f,
	.derate_severe = 0.5f,
	.vbus_invalid_ms = 200u,
	.enable_mask = {0xFFFFFFFFFFFFFFFFULL, 0xFFFFFFFFFFFFFFFFULL, 0xFFFFFFFFFFFFFFFFULL},
	/* 调试验证期: 故障级/异常级一律仅记录告警, 不执行停机/降功率
	 * (条件消失自动清除); 验证完成后改回 {0,0,0} 按码表动作执行 */
	.action_policy = {2u, 2u, 0u},
};

/* 新故障码 -> 旧 SYSTEM_FAULT_* 位映射(低13位兼容旧上位机) */
typedef struct
{
	uint16_t code;
	uint32_t bit;
} fault_compat_map_t;

static const fault_compat_map_t s_compat_map[] = {
	{FAULT_I_OVER_PEAK, SYSTEM_FAULT_OVER_CURRENT},
	{FAULT_VBUS_OVER, SYSTEM_FAULT_OVER_VOLTAGE},
	{FAULT_VBUS_UNDER, SYSTEM_FAULT_UNDER_VOLTAGE},
	{FAULT_MOTOR_OVER_SPEED, SYSTEM_FAULT_OVER_SPEED},
	{FAULT_ALGO_DIVERGE, SYSTEM_FAULT_NUMERIC},
	{FAULT_GATE_NFAULT, SYSTEM_FAULT_GATE_DRIVER},
};
#define COMPAT_MAP_COUNT (sizeof(s_compat_map) / sizeof(s_compat_map[0]))

/* ------------------------------------------------------------------ */
/*  内部工具                                                           */
/* ------------------------------------------------------------------ */

/* 历史环形缓冲更新: 同码最近 8 条内命中则刷新, 否则写新条(循环覆盖) */
static void fm_hist_update(fault_mgr_t *fm, uint16_t code, const fault_rec_t *rec, float value)
{
	fault_hist_t *h = NULL;

	for (uint16_t k = 0u; k < fm->hist_count && k < 8u; k++)
	{
		uint16_t j = (uint16_t)((fm->hist_head + FAULT_HISTORY_DEPTH - 1u - k) % FAULT_HISTORY_DEPTH);
		if (fm->hist[j].code == code)
		{
			h = &fm->hist[j];
			break;
		}
	}
	if (h == NULL)
	{
		h = &fm->hist[fm->hist_head];
		h->code = code;
		h->first_ms = rec->first_ms;
		fm->hist_head = (uint16_t)((fm->hist_head + 1u) % FAULT_HISTORY_DEPTH);
		if (fm->hist_count < FAULT_HISTORY_DEPTH)
			fm->hist_count++;
	}
	h->count = rec->count;
	h->value = value;
	h->last_ms = fm->uptime_ms;
}

/* 触发/维持一条故障(任意上下文安全: 短临界区写记录+历史) */
static void fm_set_active(fault_mgr_t *fm, uint16_t code, float value)
{
	int16_t idx = fault_code_to_index(code);
	fault_rec_t *rec;
	uint32_t primask;

	if (idx < 0 || idx >= FAULT_CODE_COUNT)
		return; /* 未定义码或越界: 忽略(防御性检查) */
	/* 三级使能掩码是统一入口，所有故障来源都在记录前受其约束。 */
	if (!fault_mgr_is_enabled(code))
		return;
	rec = &fm->rec[idx];

	primask = __get_PRIMASK();
	__disable_irq();
	if (rec->status != FAULT_STATUS_ACTIVE)
	{
		/* 首次触发: 记录激活 + 统计 */
		rec->status = FAULT_STATUS_ACTIVE;
		rec->first_ms = fm->uptime_ms;
		rec->count = 1u;
		if (fm->fault_count != 0xFFFFu)
			fm->fault_count++;
		fm->last_fault_code = (uint8_t)(idx & 0xFFu); /* 最近一次新故障位(旧语义) */
	}
	else
	{
		rec->count++;
	}
	rec->last_ms = fm->uptime_ms;
	rec->value = value;
	fm_hist_update(fm, code, rec, value);
	fm->dirty = 1u;
	__set_PRIMASK(primask);
}

/* 条件消失处理: 非停机动作自动清除, 停机类保持锁存(需 CLEAR_FAULT)
 * 判定依据级别策略覆盖后的有效动作(与 fm_eval 一致):
 * 故障级/异常级被 action_policy 覆盖为仅记录时, 条件消失即自动清除 */
static void fm_set_inactive(fault_mgr_t *fm, uint16_t code)
{
	int16_t idx = fault_code_to_index(code);
	const fault_meta_t *meta;
	fault_rec_t *rec;
	uint8_t lv, act;

	if (idx < 0 || idx >= FAULT_CODE_COUNT)
		return;
	meta = &fault_meta_table[idx];
	rec = &fm->rec[idx];

	if (rec->status != FAULT_STATUS_ACTIVE)
		return;

	fm->dirty = 1u;

	lv = (uint8_t)FAULT_META_LEVEL(meta);
	act = fm->cfg.action_policy[lv - 1u];
	if (act == 0u)
		act = (uint8_t)FAULT_META_ACTION(meta);

	if (act != FAULT_ACTION_STOP_POWER && act != FAULT_ACTION_STOP_BRAKE &&
		act != FAULT_ACTION_STOP_IDLE)
		rec->status = FAULT_STATUS_CLEARED;
}

/* 仲裁+动作执行: 遍历活动记录重算 top/derate/mask, 执行级别动作
 * 快速路径: 无活动故障且无记录变化时直接返回(空闲态 10kHz 零遍历开销) */
static void fm_eval(fault_mgr_t *fm)
{
	uint32_t compat = 0u, warn = 0u, latched = 0u, level = 0u;
	uint16_t top = 0u;
	uint16_t cnt = 0u;
	float derate = 1.0f;
	uint8_t clamp = 0u, deny = 0u;

	if (fm->active_cnt == 0u && fm->dirty == 0u)
		return; /* 空闲快速路径: 状态已是清零, 无需重算 */

	for (int i = 0; i < FAULT_CODE_COUNT; i++)
	{
		const fault_meta_t *meta = &fault_meta_table[i];
		const fault_rec_t *rec = &fm->rec[i];
		uint8_t lv, action;

		if (rec->status != FAULT_STATUS_ACTIVE)
			continue;
		/* 预留码(硬件缺失)不会有人触发, 防御性跳过 */
		if (FAULT_META_PHASE(meta) == FAULT_PHASE_RESERVED)
			continue;

		lv = (uint8_t)FAULT_META_LEVEL(meta);
		cnt++;
		if (top == 0u || fault_is_higher_priority(meta->code, top))
			top = meta->code;
		level |= (1u << (lv - 1u));

		/* 策略覆盖: 1=全部停机(保守) 2=仅记录(调试) */
		action = fm->cfg.action_policy[lv - 1u];
		if (action == 0u)
			action = (uint8_t)FAULT_META_ACTION(meta);

		switch (action)
		{
		case FAULT_ACTION_STOP_POWER:
		case FAULT_ACTION_STOP_BRAKE:
		case FAULT_ACTION_STOP_IDLE:
			latched |= (1u << (lv - 1u));
			/* 停机故障交由状态机执行(fault_check 调用点在 ISR,
			 * top_fsm_switch 在该上下文本就安全) */
			if (fm->sys != NULL)
			{
				fm->sys->motor.ref.ctrl_type = REF_CTRL_IDLE;
				top_fsm_switch(fm->sys, TOP_FSM_FAULT);
			}
			break;
		case FAULT_ACTION_DERATE:
		{
			float f = (FAULT_META_DERATE(meta) == 1u) ? fm->cfg.derate_mid : fm->cfg.derate_severe;
			if (f < derate)
				derate = f;
			break;
		}
		case FAULT_ACTION_CLAMP_POS:
			clamp = 1u;
			break;
		case FAULT_ACTION_DENY:
			deny = 1u;
			break;
		default: /* LOG/RECAL/RESET: 仅记录 */
			break;
		}

		/* 兼容掩码: 旧6故障映射低位, 其余故障级 index&15 置 bit16-31 */
		if (lv == FAULT_LEVEL_CRITICAL)
		{
			uint32_t bit = 0u;
			for (uint32_t k = 0u; k < COMPAT_MAP_COUNT; k++)
			{
				if (s_compat_map[k].code == meta->code)
				{
					bit = s_compat_map[k].bit;
					break;
				}
			}
			if (bit == 0u)
				bit = 0x10000u << ((uint32_t)i & 0x0Fu);
			compat |= bit;
		}
		else if (lv == FAULT_LEVEL_EXCEPTION)
			warn |= (1u << ((uint32_t)i & 0x0Fu));
		else
			warn |= 0x10000u << ((uint32_t)i & 0x0Fu);
	}

	fm->top_fault = top;
	fm->active_cnt = cnt;
	fm->level_active = level;
	fm->derate = derate;
	fm->clamp_active = clamp;
	fm->deny_enable = deny;
	fm->compat_mask = compat;
	fm->warn_mask = warn;
	/* 锁存语义由记录 status 承载(CLEARED 前 FAULT 态不离开),
	 * 此字段仅作"存在停机类活动故障"标志供 sync 派生 */
	fm->fault_latched = latched;
	fm->dirty = 0u;
}

/* ------------------------------------------------------------------ */
/*  对外接口                                                           */
/* ------------------------------------------------------------------ */

void fault_mgr_init(void)
{
	/* 编译期断言：确保 fault_meta_table 大小与 FAULT_CODE_COUNT 匹配
	 * 使用数组大小技巧实现编译期检查（兼容C99） */
	typedef char static_assert_fault_meta_table_size[
		(sizeof(fault_meta_table) / sizeof(fault_meta_table[0]) == FAULT_CODE_COUNT) ? 1 : -1];

	/* 保留 attach 的状态机绑定(system_state_init 先于本函数执行) */
	struct system_state_s *sys = g_fault_mgr.sys;
	memset(&g_fault_mgr, 0, sizeof(g_fault_mgr));
	g_fault_mgr.cfg = s_cfg_default;
	g_fault_mgr.derate = 1.0f;
	g_fault_mgr.sys = sys;
	if (sys != NULL)
		fault_detect_fast_init_cycles(&g_fault_mgr, ((system_state_t *)sys)->motor.dt);

	/* 看门狗复位补录(0x8101): IWDG 复位后 RAM 历史已清空,
	 * 借 RCC 标志在 init 留痕, first_ms=0 表示复位前发生 */
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
	{
		g_fault_mgr.uptime_ms = 0u;
		fm_set_active(&g_fault_mgr, FAULT_WDT_RESET, 0.0f);
	}
	__HAL_RCC_CLEAR_RESET_FLAGS();
}

void fault_mgr_attach(struct system_state_s *sys)
{
	g_fault_mgr.sys = sys;
	/* 消抖阈值按控制周期换算(ISR 拍数) */
	if (sys != NULL)
		fault_detect_fast_init_cycles(&g_fault_mgr, sys->motor.dt);
}

void fault_mgr_poll_fast(void)
{
	fault_mgr_t *fm = &g_fault_mgr;

	fm->uptime_ms = usr.sys.info.uptime_ms;
	fault_detect_fast(fm);
	fm_eval(fm);
}

void fault_mgr_poll_slow(void)
{
	fault_mgr_t *fm = &g_fault_mgr;

	fm->uptime_ms = usr.sys.info.uptime_ms;
	fault_detect_slow(fm);
	fm_eval(fm);
}

void fault_mgr_report(uint16_t code, float value)
{
	fault_mgr_t *fm = &g_fault_mgr;

	fm->uptime_ms = usr.sys.info.uptime_ms;

	/* 检查故障是否使能，未使能则不上报 */
	if (!fault_mgr_is_enabled(code))
		return;

	fm_set_active(fm, code, value);
	fm_eval(fm);
}

void fault_mgr_clear(uint8_t flags)
{
	fault_mgr_t *fm = &g_fault_mgr;

	if ((flags & FAULT_CLEAR_LATCHED) != 0u)
	{
		/* 停机类故障条件已消失的记录置 CLEARED; 仍活动的保留
		 * (状态机据此维持 FAULT 态, 与旧 fault_code!=0 拒绝清障语义一致) */
		for (int i = 0; i < FAULT_CODE_COUNT; i++)
		{
			fault_rec_t *rec = &fm->rec[i];
			if (rec->status != FAULT_STATUS_ACTIVE)
				continue;
			/* 清障请求时条件是否仍存在由检测器下拍刷新,
			 * 此处先全部放行: 下拍仍触发的会重新 ACTIVE */
			rec->status = FAULT_STATUS_CLEARED;
		}
		fm->dirty = 1u;
		fm_eval(fm);
	}
	if ((flags & FAULT_CLEAR_HISTORY) != 0u)
	{
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		memset(fm->hist, 0, sizeof(fm->hist));
		fm->hist_head = 0u;
		fm->hist_count = 0u;
		__set_PRIMASK(primask);
	}
}

float fault_mgr_get_derate(void)
{
	return g_fault_mgr.derate;
}

uint16_t fault_mgr_get_top_fault(void)
{
	return g_fault_mgr.top_fault;
}

uint16_t fault_mgr_active_count(void)
{
	return g_fault_mgr.active_cnt;
}

uint32_t fault_mgr_level_active(void)
{
	return g_fault_mgr.level_active;
}

uint32_t fault_mgr_get_warn_mask(void)
{
	return g_fault_mgr.warn_mask;
}

uint8_t fault_mgr_deny_enable(void)
{
	return g_fault_mgr.deny_enable;
}

fault_cfg_t *fault_mgr_get_cfg(void)
{
	return &g_fault_mgr.cfg;
}

/**
 * @brief 检查指定故障是否使能
 * @param code 故障码
 * @return 1=使能 0=禁用
 * @note  enable_mask[0/1/2] 按位粒度控制(64bit): bit=级别内序号, 由
 *        fault_param_generate.py 生成时直接绑定进 fault_meta_table
 *        (FAULT_META_LEVEL_BIT), 1=使能该故障 0=禁用; 未知故障默认使能
 */
uint8_t fault_mgr_is_enabled(uint16_t code)
{
	fault_cfg_t *cfg = &g_fault_mgr.cfg;

	// 查找故障码在表中的索引
	int16_t idx = fault_code_to_index(code);
	if (idx < 0 || idx >= FAULT_CODE_COUNT)
		return 1u;  // 未知故障默认使能

	// 从元数据表获取级别 (pack2[1:0])
	const fault_meta_t *meta = &fault_meta_table[idx];
	uint8_t level = FAULT_META_LEVEL(meta);

	uint8_t level_idx;
	switch (level)
	{
		case FAULT_LEVEL_CRITICAL:  level_idx = 0; break;
		case FAULT_LEVEL_EXCEPTION: level_idx = 1; break;
		case FAULT_LEVEL_WARNING:   level_idx = 2; break;
		default: return 1u;  // 未知级别默认使能
	}

	/* 位粒度: 掩码对应位=1 启用该故障, =0 禁用 (bit 编译期绑定) */
	return (uint8_t)((cfg->enable_mask[level_idx] >> FAULT_META_LEVEL_BIT(meta)) & 1u);
}

void fault_mgr_set_enable_mask(const uint64_t mask[3])
{
	fault_mgr_t *fm = &g_fault_mgr;
	if (mask == NULL)
		return;

	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	fm->cfg.enable_mask[0] = mask[0];
	fm->cfg.enable_mask[1] = mask[1];
	fm->cfg.enable_mask[2] = mask[2];
	for (int i = 0; i < FAULT_CODE_COUNT; i++)
	{
		uint8_t level = (uint8_t)FAULT_META_LEVEL(&fault_meta_table[i]);
		if (level >= FAULT_LEVEL_CRITICAL && level <= FAULT_LEVEL_WARNING &&
			mask[level - 1u] == 0u)
		{
			fm->rec[i].status = FAULT_STATUS_CLEARED;
		}
	}
	fm->dirty = 1u;
	__set_PRIMASK(primask);
	fm_eval(fm);
}

void fault_mgr_clamp_pos(float *pos)
{
	fault_mgr_t *fm = &g_fault_mgr;

	if (pos == NULL)
		return;
	if (fm->clamp_active == 0u)
		return;
	if (*pos > fm->cfg.soft_limit_max_rad)
		*pos = fm->cfg.soft_limit_max_rad;
	else if (*pos < fm->cfg.soft_limit_min_rad)
		*pos = fm->cfg.soft_limit_min_rad;
}

const fault_rec_t *fault_mgr_get_active(uint8_t idx, uint16_t *code)
{
	fault_mgr_t *fm = &g_fault_mgr;
	uint16_t seen = 0u;

	if (idx >= fm->active_cnt)
		return NULL;

	/* 按优先级序(码表升序=优先级序)遍历活动记录 */
	for (int i = 0; i < FAULT_CODE_COUNT; i++)
	{
		if (fm->rec[i].status != FAULT_STATUS_ACTIVE ||
			FAULT_META_PHASE(&fault_meta_table[i]) == FAULT_PHASE_RESERVED)
			continue;
		if (seen == idx)
		{
			if (code != NULL)
				*code = fault_meta_table[i].code;
			return &fm->rec[i];
		}
		seen++;
	}
	return NULL;
}

const fault_hist_t *fault_mgr_get_history(uint8_t idx)
{
	fault_mgr_t *fm = &g_fault_mgr;

	if (idx >= fm->hist_count)
		return NULL;
	/* 时间倒序: idx=0 为最新(写指针前一条) */
	uint16_t j = (uint16_t)((fm->hist_head + FAULT_HISTORY_DEPTH - 1u - idx) % FAULT_HISTORY_DEPTH);
	return &fm->hist[j];
}

uint16_t fault_mgr_history_count(void)
{
	return g_fault_mgr.hist_count;
}

void fault_mgr_sync_compat(struct system_state_s *sys)
{
	fault_mgr_t *fm = &g_fault_mgr;

	/* compat_mask 仅含停机类活动故障(条件消失但未清障期间记录保持
	 * ACTIVE, 掩码随之保持), 与旧 fault_code/fault_latched 语义重合:
	 * CLEAR 后条件已消失则掩码清零, 状态机允许离开 FAULT 态 */
	sys->fault_code = fm->compat_mask;
	sys->fault_latched = fm->compat_mask;
	sys->fault_count = fm->fault_count;
	sys->last_fault_code = fm->last_fault_code;
}

/* ------------------------------------------------------------------ */
/*  检测器与记录联动: fm_set_active/fm_set_inactive 供 detect 调用      */
/*  (声明于此, 定义依赖 fault_mgr 单例; detect 文件 include 本头文件     */
/*   无法访问 static, 故导出内部接口)                                   */
/* ------------------------------------------------------------------ */
void fault_mgr_internal_set(uint16_t code, float value)
{
	fm_set_active(&g_fault_mgr, code, value);
}

void fault_mgr_internal_clear(uint16_t code)
{
	fm_set_inactive(&g_fault_mgr, code);
}

