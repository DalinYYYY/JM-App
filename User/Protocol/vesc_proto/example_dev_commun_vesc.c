/*
 * example_dev_commun_vesc.c - 设备层 VESC 通信使用示例(场景: 让 VESC Tool 识别本机)
 *
 * 本文件仅作参考, 不参与固件编译。展示 dev_commun_vesc 的三处集成点。
 *
 * 连接流程:
 *   1. VESC Tool 打开串口 → 发 COMM_FW_VERSION → 本设备回复身份 → 被识别。
 *   2. VESC Tool 周期发 COMM_GET_VALUES → 本设备回填实时值 → 仪表盘显示。
 */
#include "dev_commun_vesc.h"

/* ---- 1) 仪表盘实时值数据源(用你自己的传感器/控制量替换) ---- */
static void app_fill_values(vesc_values_t *v)
{
	v->temp_fet = 32.5f;	  /* 控制器温度 ℃ */
	v->temp_motor = 28.0f;	  /* 电机温度 ℃ */
	v->current_motor = 1.2f;  /* 电机电流 A */
	v->current_in = 0.8f;	  /* 输入电流 A */
	v->duty = 0.15f;		  /* 占空比 */
	v->rpm = 1500.0f;		  /* 电气转速 ERPM */
	v->v_in = 24.3f;		  /* 输入电压 V */
	v->fault_code = 0;		  /* 无故障 */
	v->controller_id = 0;
}

/* ---- 2) 初始化(放在设备初始化阶段, 如线程/main 启动处) ---- */
void app_vesc_setup(void)
{
	dev_commun_vesc_init(&dev_commun_vesc, VESC_COMM_ID_1);
	dev_commun_vesc.set_values_cb(&dev_commun_vesc, app_fill_values);
	dev_commun_vesc.start(&dev_commun_vesc); /* 启动空闲中断+DMA接收 */
}

/* ---- 3) 周期轮询(放在主循环 / 通信线程, 周期 ~5~10ms 即可) ---- */
void app_vesc_loop(void)
{
	dev_commun_vesc.poll(&dev_commun_vesc); /* 取数据喂协议栈, 自动回复 */
}

/*
 * 第 4 处集成点已在固件内完成:
 *   Core/Src/stm32g4xx_it.c 的 USART1_IRQHandler 已调用
 *   dev_commun_vesc.on_rx_idle(&dev_commun_vesc); 检测 IDLE 并锁存本帧。
 */
