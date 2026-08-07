/**
 * @file        dev_commun_can.h
 * @brief       关节电机 CAN/CAN-FD 通信设备(承载 joint_proto 协议, 对接 drv_can 驱动)
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-07-27
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-07-27 | 1.0  | Dalin  | 初始创建 (CAN 设备粘合层) |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        分层: 本设备(粘合层) → jm_proto_can(CAN 编解码/多帧/MIT压缩) → jm_proto(命令分发)
 *                                  ↘ drv_can(FDCAN/经典 CAN 收发, 含 Bus-Off 恢复)
 *              使用方式:
 *                - 应用先 set_ops() 注入业务回调(电机控制/参数读写数据源);
 *                - 调用 start() 启动 CAN(配置双过滤器、注册回调、使能中断);
 *                - CAN 接收中断自动回调 on_rx_msg()(ISR 上下文, 仅 feed 协议层);
 *                - 主循环/线程周期调用 poll()(执行异步任务、降级超时检查);
 *                - 可选 report() 主动上报反馈帧, 无需上位机轮询。
 * @note        与 dev_commun_uart 对称设计:
 *                - jm_proto_can_t 实例内嵌为首成员, 与 UART 路径共用同一套 dispatch;
 *                - 业务回调(jm_proto_ops_t)由应用统一注入, 两路传输行为一致。
 * @note        零拷贝接收: drv_can 接收回调直接将 drvCanMsg_t 转换为 jm_can_frame_t
 *              并 feed 协议层, 中间无 ring buffer, 减少内存拷贝开销。
 * @note        双过滤器: start() 内部按 motor_id 计算单播/广播 ID 并调用
 *              drv_can_init_dual_filter(), FDCAN 同时收本机单播与广播帧。
 */
#ifndef __DEV_COMMUN_CAN_H__
#define __DEV_COMMUN_CAN_H__

#include "dev_config.h"
#if defined(USE_DEV_COMMUN_CAN)

#include "drv_can.h"      /* canNumber_e / drvCanMsg_t / drvCanDiag_t */
#include "jm_proto_can.h" /* jm_proto_can_t / jm_proto_ops_t / jm_can_frame_t */
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

	/**
	 * @brief 关节电机 CAN 通信实例编号(顺序须与 dev_config.c 的 commun_can_list 一致)
	 * @param  JM_CAN_COMM_ID_1       : 第一路 CAN 通信
	 * @param  JM_CAN_COMM_ID_MAX     : 实例数量边界
	 */
	typedef enum
	{
		JM_CAN_COMM_ID_1 = 0,
		JM_CAN_COMM_ID_MAX,
	} jm_can_comm_id_e;

	/**
	 * @brief 关节电机 CAN 通信设备配置(在 dev_config.c 的 commun_can_list 填表)
	 * @param  name                   : 实例名(调试用)
	 * @param  can                    : 所属 CAN 编号(须为已 CubeMX 初始化的 FDCAN/CAN)
	 * @param  motor_id               : 本机地址(1~127; 0 保留为广播)
	 * @param  ide                    : 0=标准帧(11位ID), 1=扩展帧(29位ID)
	 * @param  use_fd                 : 1=启用 FD 帧(>8字节载荷走 FD), 0=仅经典帧
	 * @note   use_fd 仅在 USE_CAN_FD_MODE=1 且 JM_PERIPH_CAN_FD 时生效;
	 *         经典 CAN 或 FD 关闭时强制走 8 字节单帧/多帧分包。
	 */
	typedef struct
	{
		char name[20];
		canNumber_e can;
		uint8_t motor_id;
		uint8_t ide;
		uint8_t use_fd;
	} dev_commun_can_config_t;

	/* 配置表定义在 dev_config.c */
	extern const dev_commun_can_config_t commun_can_list[JM_CAN_COMM_ID_MAX];

/* 一帧最大经典 CAN 传输需 ceil((JM_PAYLOAD_MAX + CRC16) / 7) 帧;
 * 环形队列多留 1 个槽位, 因为 head == tail 表示队列空 */
#define DEV_COMMUN_CAN_RX_QUEUE_SIZE 40u

	typedef struct
	{
		jm_can_frame_t frame;
		uint32_t tick;
	} dev_commun_can_rx_item_t;

	/**
	 * @brief 关节电机 CAN 通信设备对象
	 * @param  jm                     : joint_proto CAN 协议实例(私有, 须为首成员)
	 * @param  can                    : 所属 CAN 编号(私有)
	 * @param  motor_id               : 本机地址(私有)
	 * @param  ide                    : 帧类型(0标准/1扩展, 私有)
	 * @param  use_fd                 : 是否启用 FD 帧(板级配置, 私有)
	 * @param  use_fd_runtime         : 运行期 FD 模式(由 0xF3 命令切换, 私有, 默认0=经典)
	 * @param  started                : 接收已启动标志(私有)
	 * @param  last_rx_tick           : 最近收到 CAN 帧(含广播)的系统 tick ms
	 * @param  last_error             : 最近一次错误码(<0=内部错误, >0=drv_can 返回)
	 * @param  diag                   : 诊断统计快照(poll 时刷新, 供应用读取)
	 * @param  ops                    : 业务回调(应用注入, start 前须设置)
	 * @param  set_ops                : 注入业务回调数据源
	 * @param  start                  : 启动 CAN(双过滤器+回调注册+中断使能), 返回 DEV_EOK/DEV_ERROR
	 * @param  on_rx_msg              : CAN 接收 ISR 中调用, 把 drvCanMsg_t 喂入协议层
	 * @param  on_err                 : CAN 错误 ISR 中调用(Bus-Off/错误被动等)
	 * @param  poll                   : 主循环/线程周期调用(降级超时检查+诊断刷新)
	 * @param  report                 : 主动上报一帧(cmd+载荷), 无需上位机轮询
	 * @param  check_loss             : 检查通信中断降级, 返回 1=已超时需 IDLE
	 */
	typedef struct dev_commun_can
	{
		jm_proto_can_t jm;      /* 须为首成员, 与 jm_proto_dispatch 兼容 */
		canNumber_e can;
		uint8_t motor_id;
		uint8_t ide;
		uint8_t use_fd;          /* 板级配置(编译期) */
		uint8_t use_fd_runtime;  /* 运行期模式(由 0xF3 命令切换, 默认0=经典) */
		uint8_t started;
		uint8_t can_uid[12];
		uint8_t can_di56[7];
		uint8_t can_di_guard;
		uint8_t can_di_valid;
		uint8_t discover_pending;
		uint8_t id_switch_pending;
		uint8_t pending_new_id;
		uint32_t discover_due_tick;
		uint32_t commissioning_quiet_until;
		volatile uint8_t rx_queue_head;
		volatile uint8_t rx_queue_tail;
		dev_commun_can_rx_item_t rx_queue[DEV_COMMUN_CAN_RX_QUEUE_SIZE];

		uint32_t last_rx_tick;  /* 通信中断降级计时基准 */
		int last_error;

		/* 诊断/统计 */
		uint32_t init_count;
		uint32_t start_count;
		uint32_t start_fail_count;
		uint32_t poll_count;
		uint32_t poll_not_started_count;
		uint32_t tx_count;
		uint32_t tx_fail_count;
		uint32_t rx_irq_count;
		uint32_t rx_queue_overflow_count;
		uint32_t err_irq_count;
		uint32_t loss_timeout_count; /* 触发降级的次数 */
		drvCanDiag_t diag;           /* drv_can 诊断快照 */

		/* public */
		const jm_proto_ops_t *ops;

		void (*set_ops)(struct dev_commun_can *pobj, const jm_proto_ops_t *ops);
		int (*start)(struct dev_commun_can *pobj);
		void (*on_rx_msg)(struct dev_commun_can *pobj, canNumber_e can, drvCanMsg_t *msg);
		void (*on_err)(struct dev_commun_can *pobj, canNumber_e can, uint8_t error_type);
		void (*poll)(struct dev_commun_can *pobj);
		void (*report)(struct dev_commun_can *pobj, uint8_t cmd, const uint8_t *body, uint16_t len);
		int (*check_loss)(struct dev_commun_can *pobj, uint32_t now_tick);
	} dev_commun_can_t;

	/**
	 * @brief       初始化关节电机 CAN 通信设备对象(按 id 取配置表组装)
	 * @param        pobj             : 设备对象
	 * @param        id               : 实例编号(对应 commun_can_list 表项)
	 * @note         init 后须 set_ops() 注入业务回调, 再 start()。
	 */
	void dev_commun_can_init(dev_commun_can_t *pobj, jm_can_comm_id_e id);

	extern dev_commun_can_t dev_commun_can;

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_COMMUN_CAN */
#endif /* __DEV_COMMUN_CAN_H__ */
