#ifndef __MCU_COMPAT_H__
#define __MCU_COMPAT_H__

/*
 * MCU 兼容判定头（单一入口）
 * 以 Keil 工程预定义的 HAL 标准宏（-D 编译器选项）为唯一事实源,
 * 统一 User/ 下的 MCU 条件编译。
 *
 * 所有需要按 MCU 切换的代码 #include 本头, 使用 JM_MCU_F4 / JM_MCU_G4 等判定,
 * 禁止直接使用 STM32F4 / STM32G4 系列宏（依赖 HAL 头包含顺序, 不可靠）
 * 或 STM32F405 / STM32G474 无后缀宏（不匹配 HAL 标准）。
 *
 * 支持系列（Flash 驱动类型）:
 *   - JM_MCU_F1:  STM32F103xx 等           → Flash: 页擦除+halfword (需 drv_flash_f1.c, 预留)
 *   - JM_MCU_F3:  STM32F302xx/F303xx 等    → Flash: 页擦除+halfword (需 drv_flash_f3.c, 预留)
 *   - JM_MCU_F4:  STM32F405xx/F407xx 等    → Flash: 扇区擦除+word   (drv_flash_f4.c)
 *   - JM_MCU_G4:  STM32G474xx/G473xx 等    → Flash: 页擦除+doubleword (drv_flash_g4.c)
 *   - JM_MCU_L4:  STM32L431xx/L476xx 等    → Flash: 页擦除+doubleword (复用 drv_flash_g4.c, 接口一致)
 *   - JM_MCU_H7:  STM32H745xx/H747xx 等    → Flash: 扇区擦除+256bit (需 drv_flash_h7.c, 预留)
 *
 * 添加新系列: 在下方加一行 #elif 分支, 在 drv_config.h 选对应 Flash 驱动即可。
 */

/*=== MCU 系列判定 ===*/
#if defined(STM32F103xB) || defined(STM32F103xC) || defined(STM32F103xD) || defined(STM32F103xE) || \
    defined(STM32F102xB) || defined(STM32F102xC)
#define JM_MCU_F1
#elif defined(STM32F302xC) || defined(STM32F302xE) || \
      defined(STM32F303xC) || defined(STM32F303xE) || defined(STM32F334x8)
#define JM_MCU_F3
#elif defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
      defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || defined(STM32F439xx) || \
      defined(STM32F411xE) || defined(STM32F401xC) || defined(STM32F401xE)
#define JM_MCU_F4
#elif defined(STM32G474xx) || defined(STM32G473xx) || defined(STM32G484xx) || \
      defined(STM32G431xx) || defined(STM32G441xx) || defined(STM32G491xx)
#define JM_MCU_G4
#elif defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || \
      defined(STM32L452xx) || defined(STM32L476xx) || defined(STM32L475xx)
#define JM_MCU_L4
#elif defined(STM32H745xx) || defined(STM32H747xx) || defined(STM32H743xx) || defined(STM32H750xx)
#define JM_MCU_H7
#endif

#if !defined(JM_MCU_F1) && !defined(JM_MCU_F3) && !defined(JM_MCU_F4) && \
    !defined(JM_MCU_G4) && !defined(JM_MCU_L4) && !defined(JM_MCU_H7)
#error "Unknown MCU. Define STM32F405xx / STM32G474xx / STM32F103xx etc. in Keil target options."
#endif

/*=== 外设接口分组（跨系列共性, 供 drv_can/drv_adc 等使用） ===*/

/* CAN vs FDCAN: F1/F3/F4 用经典 CAN, G4/L4/H7 用 FDCAN */
#if defined(JM_MCU_F1) || defined(JM_MCU_F3) || defined(JM_MCU_F4)
#define JM_PERIPH_CAN_CLASSIC
#elif defined(JM_MCU_G4) || defined(JM_MCU_L4) || defined(JM_MCU_H7)
#define JM_PERIPH_CAN_FD
#endif

/* ADC 校准接口分组:
 * - G4/L4/H7: 双参 HAL_ADCEx_Calibration_Start(handle, mode)
 * - F1/F3:    单参 HAL_ADCEx_Calibration_Start(handle)
 * - F4:       无校准接口(F4 ADC 硬件不支持软件校准), 不定义任何宏, 编译为空 */
#if defined(JM_MCU_G4) || defined(JM_MCU_L4) || defined(JM_MCU_H7)
#define JM_PERIPH_ADC_CALIB_DUAL_PARAM
#elif defined(JM_MCU_F1) || defined(JM_MCU_F3)
#define JM_PERIPH_ADC_CALIB_SINGLE_PARAM
#endif

#endif /* __MCU_COMPAT_H__ */
