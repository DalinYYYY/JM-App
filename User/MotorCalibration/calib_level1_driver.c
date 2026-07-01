/**
 * @file calib_level1_driver.c
 * @brief L1 驱动硬件底层校准（ADC偏置/增益/电流传感器/温度/母线电压/死区特性）
 * @note 桩实现：所有子模式 poll 直接返回 DONE。真实算法后续逐个填充。
 *       添加新子模式：在 start/poll 的 switch 中各加一个 case。
 */
#include "calib_types.h"

static uint8_t s_submode;
static motor_param_t *s_param;
static float s_dt;

/* ---- 各子模式的独立实现函数（桩） ---- */
static calib_state_e poll_adc_offset(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_adc_gain(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_current_sensor(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_temp_sensor(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_vbus(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_deadtime(void)
{
	return CALIB_STATE_DONE;
}

static bool calib_level1_start(uint8_t submode, motor_param_t *param, float dt)
{
	s_param = param;
	s_dt = dt;
	s_submode = submode;

	switch (submode)
	{
		case CALIB_L1_ADC_OFFSET:
		case CALIB_L1_ADC_GAIN:
		case CALIB_L1_CURRENT_SENSOR:
		case CALIB_L1_TEMP_SENSOR:
		case CALIB_L1_VBUS:
		case CALIB_L1_DEADTIME:
			/* TODO: 按 submode 初始化对应校准（注入电压/采样配置等） */
			return true;
		default:
			return false; /* 不支持的子模式 */
	}
}

static calib_state_e calib_level1_poll(void)
{
	switch (s_submode)
	{
		case CALIB_L1_ADC_OFFSET: return poll_adc_offset();
		case CALIB_L1_ADC_GAIN: return poll_adc_gain();
		case CALIB_L1_CURRENT_SENSOR: return poll_current_sensor();
		case CALIB_L1_TEMP_SENSOR: return poll_temp_sensor();
		case CALIB_L1_VBUS: return poll_vbus();
		case CALIB_L1_DEADTIME: return poll_deadtime();
		default: return CALIB_STATE_FAILED;
	}
}

static void calib_level1_abort(void)
{
	/* TODO: 按 s_submode 停止对应校准 */
}

const calib_level_ops_t calib_level1_ops = {
	.start = calib_level1_start,
	.poll = calib_level1_poll,
	.abort = calib_level1_abort,
};
