
/**
 * @file        dev_motor.h
 * @brief 		电机实例化：编码器+多圈计数+FOC+PWM+相电流采样
 * 
 * @author      Dalin (dalin@robot.com)
 * @version     1.0
 * @date        2026-06-17
 * 
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 * 
 * 
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-17     | 1.0  | yangsl | 初始创建   |
 * 
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#include "motor_loop_config.h"

/* 与 dev_motor_virtual.c 对称：宏=1（真实驱动）时本文件编译为实体，
 * 宏=0（虚拟电机）时整体编译为空，避免 dev_motor_init 等符号与
 * dev_motor_virtual.o 重复定义（L6200E）。切换只改宏，无需动 Keil 工程。 */
#if (MOTOR_LOOP_ENABLE_DEV_DRIVER)

#include "dev_motor.h"
#include "assert_report.h"
#include "runtime_param.h" /* usr.motor_param：读取已标定的 enc_direction/enc_offset */

static dev_motor_enable_config_t motor_enable_list[DEV_MOTOR_MAX] = {
	{"MOTOR1_EN", {(gpioType_e)DRV_GPIOB, (gpioPin_e)DRV_PIN_2, (drvPinState_e)0}},
};

/* 使能电机功率级(拉高EN引脚) */
static void dev_motor_enable(void)
{
	drv_gpio_write(motor_enable_list[DEV_MOTOR_1].gpio, (drvPinState_e)1);
}

/* 禁用电机功率级(拉低EN引脚) */
static void dev_motor_disable(void)
{
	drv_gpio_write(motor_enable_list[DEV_MOTOR_1].gpio, (drvPinState_e)0);
}

/* 设备角度补偿回调(注入多圈计数) TODO: 接入 motor_info 后返回实际补偿值 */
static float device_compensation(void)
{
	return 0.0F;
}

/* 获取电机极对数 TODO: 后续支持自动识别 */
static uint8_t dev_motor_get_poles(motor_id_e id)
{
	uint8_t poles;
	switch (id)
	{
		case DEV_MOTOR_1: poles = 7; break;
		default: poles = 7; break;
	}
	return poles;
}

/*============================================================================
 * 抽象编码器适配层
 *   把具体芯片(mt6701/mt6835/...)适配到 dev_encoder_t 抽象接口。
 *   控制层只调 encoder.update / encoder.get_mechanical_angle，不感知型号。
 *   只编译 DEV_MOTOR_ENCODER_TYPE 选中型号的那套适配函数。
 *==========================================================================*/

#if (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6701)
/** @brief MT6701 → 抽象编码器：刷新并同步机械角到 enc->mechanical_angle */
static void encoder_mt6701_update(struct dev_encoder *enc)
{
	dev_mt6701_t *chip = (dev_mt6701_t *)enc->ctx;
	chip->update(chip);
	enc->mechanical_angle = chip->mechanical_angle; // mt6701 无 getter，直接读字段
}

static float encoder_mt6701_get_mechanical_angle(struct dev_encoder *enc)
{
	return enc->mechanical_angle;
}

#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6835)
/** @brief MT6835 → 抽象编码器：刷新并经 get_mechanical_angle 取角 */
static void encoder_mt6835_update(struct dev_encoder *enc)
{
	dev_mt6835_t *chip = (dev_mt6835_t *)enc->ctx;
	chip->update(chip);
	enc->mechanical_angle = chip->get_mechanical_angle(chip);
}

static float encoder_mt6835_get_mechanical_angle(struct dev_encoder *enc)
{
	return enc->mechanical_angle;
}
#endif

void dev_motor_init(dev_motor_t *pobj,
                    motor_id_e id,
                    focCurrent_t (*current_callback)(void),
                    float (*ele_radian_callback)(void))
{
	assert_report(pobj != NULL);
	assert_report(current_callback != NULL);
	assert_report(ele_radian_callback != NULL);
	assert_report(id < DEV_MOTOR_MAX);
	memset(pobj, 0, sizeof(dev_motor_t));

	dev_motor_disable(); // 默认上电禁能

	pobj->id = id;
	pobj->poles = dev_motor_get_poles((motor_id_e)id);

	pobj->fsm_tim = DRV_TIM2; // 定时器2 // TODO: 后续支持配置表

							  // 初始化编码器（型号由 DEV_MOTOR_ENCODER_TYPE 选择，控制层不感知）
#if (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6701)
	dev_mt6701_init(&pobj->mt6701, (mt6701_id_e)id);
	/* 从已标定参数加载编码器零位和方向（启动时 Flash 参数已加载到 usr.motor_param）*/
	{
		const encoder_param_t *enc_cfg = &usr.motor_param[(motor_num_e)id].encoder_param;

		/* enc_offset 已为角度值(deg)，直接写入 dev_mt6701 */
		pobj->mt6701.offset = enc_cfg->enc_offset;
		pobj->mt6701.dir = (enc_cfg->enc_direction < 0) ? MT6701_DIR_CCW : MT6701_DIR_CW;
	}

	/* 装配抽象编码器接口 → MT6701 */
	pobj->encoder.ctx = &pobj->mt6701;
	pobj->encoder.update = encoder_mt6701_update;
	pobj->encoder.get_mechanical_angle = encoder_mt6701_get_mechanical_angle;

#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6835)
	dev_mt6835_init(&pobj->mt6835, (mt6835_id_e)id);
	pobj->mt6835.set_zero_angle(&pobj->mt6835, 0.0f); // TODO: 后续支持配置表
	pobj->mt6835.set_dir(&pobj->mt6835, 0);           // TODO: 后续支持配置表

	/* 装配抽象编码器接口 → MT6835 */
	pobj->encoder.ctx = &pobj->mt6835;
	pobj->encoder.update = encoder_mt6835_update;
	pobj->encoder.get_mechanical_angle = encoder_mt6835_get_mechanical_angle;

#else
#error "未知的 DEV_MOTOR_ENCODER_TYPE，请在 dev_motor.h 选择支持的编码器型号"
#endif

	// 初始化角度转化器（仅角度/速度，不含多圈）
	motion_param_init(&pobj->motor_param, pobj->poles, 10, NULL); // TODO: 后续支持配置表

	// 初始化绝对多圈计数（单编码器软件累圈，设备补偿回调注入）
	multiturn_config_t mt_cfg;
	memset(&mt_cfg, 0, sizeof(mt_cfg));
	mt_cfg.mode = MULTITURN_MODE_SOFT;
	mt_cfg.device_compensation_callback = device_compensation;
	multiturn_init(&pobj->multiturn, &mt_cfg);

	// 初始化半桥驱动器
	dev_half_bridge_init(&pobj->half_bridge, (half_bridge_id_e)id);

	// 初始化三相adc电流采样
	dev_phase_current_init(&pobj->phase_current, PHASE_CURRENT_GAIN, PHASE_CURRENT_SHUNT);
	pobj->phase_current.set_offset(&pobj->phase_current,
	                               (dev_current_i3axis_t){PHASE_CURRENT_ZERO_ADC,
	                                                      PHASE_CURRENT_ZERO_ADC,
	                                                      PHASE_CURRENT_ZERO_ADC}); // TODO: 后续支持自动校准

	// 初始化FOC
	pobj->current_callback = current_callback;
	pobj->ele_radian_callback = ele_radian_callback;
	foc_init(&pobj->foc, pobj->current_callback, pobj->ele_radian_callback);

	dev_motor_enable();
}

void dev_motor_set_encoder_dir(dev_motor_t *pobj, int8_t dir)
{
	assert_report(pobj != NULL);
	if (dir != 1 && dir != -1)
	{
		return; /* 仅接受 1(CW) / -1(CCW) */
	}

#if (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6701)
	mt6701_dir_e hw_dir = (dir < 0) ? MT6701_DIR_CCW : MT6701_DIR_CW;
	pobj->mt6701.set_dir(&pobj->mt6701, hw_dir);
#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6835)
	pobj->mt6835.set_dir(&pobj->mt6835, (dir < 0) ? 1 : 0);
#endif

	/* 同步到参数层(便于后续持久化到 Flash / 上位机读取一致) */
	motor_param_set_enc_direction(&usr.motor_param[(motor_num_e)pobj->id], dir);
}

#endif /* MOTOR_LOOP_ENABLE_DEV_DRIVER */
