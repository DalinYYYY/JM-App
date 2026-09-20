/**
 * @file    cogging_comp.h
 * @brief   齿槽转矩补偿（查表前馈 + Flash 表存储，纯查表热路径 ISR 安全）
 * @note    齿槽转矩 T_cog(θm) 为机械角度周期函数，标定表存"补偿电流(mA，
 *          按标定时 Kt)"，运行时按机械角线性插值前馈。表由 L5.1 标定生成
 *          （正反双程恒速扫描，正反平均自动抵消库仑摩擦），存独立 Flash 区
 *          （COGGING_FLASH_* 板级宏，默认 motor_info 扇区紧邻下方）。
 *
 *          模块裁剪：COGGING_COMP_EN=0 时函数替换为恒 0 static inline，
 *          调用方无需条件编译，常量折叠后零 ROM/RAM/时间开销。
 *          实时性：current() 纯算术无阻塞（约 35 周期 @M4F），可在 ISR/控制环
 *          调用；reload()/flash_write() 含 Flash 擦写阻塞，严禁在控制环调用。
 *          通用性：全角度均匀索引，不假设极槽配合（1024 点级分辨率对
 *          LCM≤252 的常见极槽配合全谐波覆盖，见方案文档覆盖矩阵）。
 */

#ifndef __COGGING_COMP_H__
#define __COGGING_COMP_H__

#include <stdint.h>
#include "dev_config.h" /* USE_DEV_FLASH 门控 + 板级 COGGING_FLASH_* 覆盖 */

#if defined(USE_DEV_FLASH)

/* ===== 模块裁剪开关：1=启用 0=裁剪（输出恒 0）===== */
#ifndef COGGING_COMP_EN
#define COGGING_COMP_EN 1
#endif

/* ===== 表参数（存储布局与查表环绕强耦合，勿随意改动）===== */
#define COGGING_TABLE_N     1008u                          /* 点数（非 2 幂，环绕用条件修正）*/
#define COGGING_TABLE_BYTES (COGGING_TABLE_N * 2u)         /* int16 表字节数 2016B */
#define COGGING_MAGIC       0x31474743u                    /* "CGG1" 小端 */
#define COGGING_VER         1u

/* ===== 存储区板级配置（dev_config_board.h 可按板覆盖）=====
 * 默认 G474 双 Bank: 0x0804E000 单页 2KB 直写（无磨损均衡, 标定写入频率
 * 极低无需轮转; 页内擦写寿命 1 万次远超标定次数）。该页占用 Bank2 末 8KB
 * 的首 2KB, 其余 6KB 由链接脚本(.sct)声明为执行域还给代码区。
 * Keil ER_IROM2 须避开本页（见各板 .sct）。
 * ODrive(F405) 覆盖为独立大扇区（如 Sector10 0x080C0000/128KB 单扇区）。*/
#ifndef COGGING_FLASH_START_ADDR
#define COGGING_FLASH_START_ADDR 0x0804E000U
#endif
#ifndef COGGING_FLASH_TOTAL_SIZE
#define COGGING_FLASH_TOTAL_SIZE 0x00000800U /* 2KB 单页直写 */
#endif
#ifndef COGGING_FLASH_PAGE_SIZE
#define COGGING_FLASH_PAGE_SIZE 2048U
#endif

/* ===== 存储记录布局（2040B 页数据区上限内，共 2032B）=====
 *   u64 [0]: 头部 magic32("CGG1") | table_n<<32 | ver<<48
 *   u64 [1]: crc32(低32, 覆盖全部 254 u64、本字段置零计算) | 保留高32
 *   252 u64: 1008 × int16 补偿电流(mA, 按标定时 Kt)                     */

#if COGGING_COMP_EN

/**
 * @brief  从 Flash 加载齿槽表并校验（magic/点数/CRC），更新 RAM 表与有效标志
 * @return 1=表有效已加载, 0=无有效表（RAM 表清零, current 恒返回 0）
 * @note   阻塞（Flash 读 ~µs 级，无擦写）。初始化/标定写表后调用。
 */
uint8_t cogging_comp_reload(void);

/**
 * @brief  标定表落盘：整块擦写 Flash + 回读 CRC 校验 + 自动 reload
 * @param  table_ma 1008 点补偿电流(mA)
 * @return 1=成功, 0=写入/校验失败（RAM 表保持旧状态）
 * @note   **阻塞**：内部关中断擦写约 10~40ms，仅可在标定流程（非控制环）调用。
 */
uint8_t cogging_comp_flash_write(const int16_t *table_ma);

/**
 * @brief  ISR 安全的待写表登记：拷贝表到静态缓冲并置待写标志（不碰 Flash）
 * @param  table_ma 1008 点补偿电流(mA)
 * @note   可在标定 ISR 调用；实际擦写由 cogging_comp_flush() 在线程上下文完成。
 */
void cogging_comp_request_write(const int16_t *table_ma);

/**
 * @brief  待写表落盘（若有登记）：擦写 Flash + 校验 + reload
 * @return 1=本次完成落盘且校验通过, 0=无待写或写失败
 * @note   **阻塞**（Flash 擦写 10~40ms）：仅限线程上下文（如 idle 线程）周期调用，
 *         严禁在 ISR/控制环调用。Flash 擦写期间该 Bank 不能取指，ISR 上下文
 *         调用会与 Bank2 上的代码执行冲突（RM0440 禁止）。
 *         结果可经 cogging_comp_get_write_state() 查询（诊断落盘链路）。
 */
uint8_t cogging_comp_flush(void);

/**
 * @brief  查询表落盘状态（诊断用）
 * @return 0=空闲无操作, 1=待写, 2=上次落盘成功, 3=上次落盘失败
 * @note   读后不清除，需显式 cogging_comp_clear_write_state() 复位。
 *         线程上下文在落盘完成后将状态回写到使能参数（失败自动关补偿）。
 */
uint8_t cogging_comp_get_write_state(void);

/**
 * @brief  清除落盘状态（回到空闲）
 */
void cogging_comp_clear_write_state(void);

/**
 * @brief  表有效标志（RAM 表已通过校验加载）
 */
uint8_t cogging_comp_is_valid(void);

/**
 * @brief  查表前馈电流：线性插值 + 增益，theta 环绕 mod 2π
 * @param  theta_mech 编码器机械单圈角(rad, 与 L5.1 标定分桶同源:
 *                    encoder.mechanical_angle×π/180)。勿传多圈累计位置——
 *                    其零点为上电位姿且可被 reset_position 清零, 会整体错相
 * @param  enable     运行期使能（PID184, 0=恒返回 0）
 * @param  gain       补偿增益（PID185, 0~1.5; 0=恒返回 0）
 * @return 前馈电流(A)；表无效/未使能/gain=0 时恒 0
 * @note   纯算术 ~35 周期 @M4F, 可在 ISR/控制环调用。
 */
float cogging_comp_current(float theta_mech, uint8_t enable, float gain);

#else /* 裁剪：恒 0 内联，零开销 */

static inline uint8_t cogging_comp_reload(void) { return 0u; }
static inline uint8_t cogging_comp_flash_write(const int16_t *table_ma)
{
	(void)table_ma;
	return 0u;
}
static inline void cogging_comp_request_write(const int16_t *table_ma)
{
	(void)table_ma;
}
static inline uint8_t cogging_comp_flush(void) { return 0u; }
static inline uint8_t cogging_comp_get_write_state(void) { return 0u; }
static inline void cogging_comp_clear_write_state(void) {}
static inline uint8_t cogging_comp_is_valid(void) { return 0u; }
static inline float cogging_comp_current(float theta_mech, uint8_t enable, float gain)
{
	(void)theta_mech;
	(void)enable;
	(void)gain;
	return 0.0f;
}

#endif /* COGGING_COMP_EN */

#endif /* USE_DEV_FLASH */

#endif /* __COGGING_COMP_H__ */
