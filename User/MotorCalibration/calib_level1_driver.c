/**
 * @file calib_level1_driver.c
 * @brief L1 驱动硬件底层校准（ADC偏置/增益/电流传感器/温度/母线电压/死区特性）
 * @note L1.1 ADC 偏置标定已实现（阻塞式 1000 次采样取均值）。
 *       L1.2-6 仍为桩实现，后续逐个填充。
 *       添加新子模式：在 start/poll 的 switch 中各加一个 case。
 */
#include "calib_types.h"
#include "calib_config.h"
#include "calib_mgr.h"
#include "dev_motor.h"
#include "motor_param.h"
#include "calib_step.h"

static uint8_t s_submode;
static motor_param_t *s_param;
static float s_dt;

/* ===================== L1.1 ADC 偏置标定 =====================
 * 阻塞式：调用 m->phase_current.calibrate_offset() 采样 N 次取均值。
 * 须在电机不通电、相电流为 0 时调用（L7 序列中 L1.1 排在所有施加电压的标定之前）。
 * 采样数 CALIB_CFG_L1_ADC_OFFSET_SAMPLES=1000，dt=100us，约耗时 100ms。
 * ========================================================================= */
static calib_state_e poll_adc_offset(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;

	/* 阻塞式采样：calibrate_offset 内部循环 N 次 dev_phase_current_get_value */
	m->phase_current.calibrate_offset(&m->phase_current, CALIB_CFG_L1_ADC_OFFSET_SAMPLES);

	calib_mgr_mark_done(CALIB_LEVEL1_DRIVER, CALIB_L1_ADC_OFFSET);
	return CALIB_STATE_DONE;
}

/* ---- L1.2-6 桩实现 ---- */
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
}

const calib_level_ops_t calib_level1_ops = {
	.start = calib_level1_start,
	.poll = calib_level1_poll,
	.abort = calib_level1_abort,
};
