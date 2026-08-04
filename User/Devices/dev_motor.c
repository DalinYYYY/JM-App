
/**
 * @file        dev_motor.h
 * @brief 		电机实例化：编码器+多圈计数+FOC+PWM+相电流采样
 * 
 * @author      Dalin (dalinyy@163.com)
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

/* 编码器适配层头（按型号条件包含，替代原 dev_motor.c 内联的适配函数）*/
#if (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6701)
#include "dev_encoder_mt6701.h"
#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6835)
#include "dev_encoder_mt6835.h"
#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_AS5047)
#include "dev_encoder_as5047.h"
#endif

#if defined(JM_BOARD_ODRIVE)
/* ODrive Mini: EN_GATE = PB12 (DRV8301) */
static dev_motor_enable_config_t motor_enable_list[DEV_MOTOR_MAX] = {
	{"MOTOR1_EN", {(gpioType_e)DRV_GPIOB, (gpioPin_e)DRV_PIN_12, (drvPinState_e)0}},
};
#else
/* SFOC/V1: EN = PB2 */
static dev_motor_enable_config_t motor_enable_list[DEV_MOTOR_MAX] = {
	{"MOTOR1_EN", {(gpioType_e)DRV_GPIOB, (gpioPin_e)DRV_PIN_12, (drvPinState_e)0}},
};
#endif

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
 *   适配函数已迁移到 dev_encoder_mt6701.c / dev_encoder_mt6835.c，
 *   本文件通过工厂函数 dev_encoder_xxx_create() 装配，不再内联实现。
 *==========================================================================*/

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

							  /* 创建编码器实例并装配抽象接口（1行替代原30行 #if 装配）
	 * 适配层内部持有 static 实体，绑定到 encoder.ctx 并装配全部方法指针 */
#if (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6701)
	dev_encoder_mt6701_create(&pobj->encoder, (mt6701_id_e)id);
#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6835)
	dev_encoder_mt6835_create(&pobj->encoder, (mt6835_id_e)id);
#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_AS5047)
	dev_encoder_as5047_create(&pobj->encoder, (as5047_id_e)id);
#else
#error "未知的 DEV_MOTOR_ENCODER_TYPE，请在 dev_motor.h 选择支持的编码器型号"
#endif

	/* 通过抽象层从已标定参数加载编码器零位和方向
	 * （启动时 Flash 参数已加载到 usr.motor_param，统一 -1/1 方向约定）*/
	{
		const motor_param_t *mp = &usr.motor_param[(motor_num_e)id];
		const encoder_param_t *enc_cfg = &mp->encoder_param;
		pobj->encoder.set_offset(&pobj->encoder, enc_cfg->enc_offset);
		pobj->encoder.set_dir(&pobj->encoder, enc_cfg->enc_direction);
		/* 极对数从已标定/Flash 参数加载，替代 dev_motor_get_poles() 的硬编码 7。
		 * motor_loop_init 已先执行 motor_profile_apply_param + sync_to_param，
		 * 故此处 motor_base.pole_pairs 为标定值(或 profile 默认值兜底)。
		 * 【关键修复】FOC 电角度 = 机械角 × poles(motion_param.c)。此前 poles 恒为 7，
		 * 与真实极对数(如 RS03=21)不符：转子一动，FOC 电角度按 7 倍算、真实按 21 倍走，
		 * 比例失配导致开环施加 UQ 有电流却无有效力矩。与 dev_motor_virtual 的 poles
		 * 同步逻辑对称(dev_motor_virtual.c:281)。*/
		if (mp->motor_base.pole_pairs != 0u)
			pobj->poles = mp->motor_base.pole_pairs;
	}

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

	/* 通过抽象编码器层设置方向（统一 -1/1 约定，适配层内部处理具体芯片差异）*/
	pobj->encoder.set_dir(&pobj->encoder, dir);

	/* 同步到参数层(便于后续持久化到 Flash / 上位机读取一致) */
	(&usr.motor_param[(motor_num_e)pobj->id])->encoder_param.enc_direction = dir;
}

#endif /* MOTOR_LOOP_ENABLE_DEV_DRIVER */
