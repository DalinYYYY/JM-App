/**
 * @file        utils.h
 * @brief       通用数学与信号处理工具集接口（角度、三角函数、FFT、滤波、常量与宏）
 *
 * @details     面向 Cortex-M4F 单精度 FPU：内联函数与常量宏均带 f 后缀，避免
 *              单精度操作数被隐式提升为双精度（编译告警 rgba(48, 34, 75, 0.33)-D）。其中数学常量
 *              （M_PI 等）保留双精度定义，在单精度场景由调用方以 (float) 强转或
 *              用宏内的 (float)(...) 编译期常量表达式完成转换。
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-06-17
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#ifndef UTILS_MATH_H_
#define UTILS_MATH_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* 函数声明（实现与详细注释见 utils.c） */
float utils_map_angle(float angle, float min, float max);                                                      // 角度映射到 [0,1]
void utils_deadband(float *value, float tres, float max);                                                      // 死区处理
float utils_angle_difference(float angle1, float angle2);                                                      // 角度差（度，±180）
float utils_angle_difference_rad(float angle1, float angle2);                                                  // 角度差（弧度，±pi）
float utils_avg_angles_rad_fast(float *angles, float *weights, int angles_num);                                // 角度加权平均
float utils_middle_of_3(float a, float b, float c);                                                            // 三浮点取中值
int utils_middle_of_3_int(int a, int b, int c);                                                                // 三整数取中值
float utils_fast_atan2(float y, float x);                                                                      // 快速 atan2
void utils_fast_sincos(float angle, float *sin, float *cos);                                                   // 快速 sin/cos
void utils_fast_sincos_better(float angle, float *sin, float *cos);                                            // 高精度快速 sin/cos
float utils_min_abs(float va, float vb);                                                                       // 取幅值较小者
float utils_max_abs(float va, float vb);                                                                       // 取幅值较大者
void utils_byte_to_binary(int x, char *b);                                                                     // 字节转二进制字符串
float utils_throttle_curve(float val, float curve_acc, float curve_brake, int mode);                           // 油门曲线整形
uint32_t utils_crc32c(uint8_t *data, uint32_t len);                                                            // CRC32C 校验
void utils_fft32_bin0(float *real_in, float *real, float *imag);                                               // 32 点 DFT 第 0 频点
void utils_fft32_bin1(float *real_in, float *real, float *imag);                                               // 32 点 DFT 第 1 频点
void utils_fft32_bin2(float *real_in, float *real, float *imag);                                               // 32 点 DFT 第 2 频点
void utils_fft16_bin0(float *real_in, float *real, float *imag);                                               // 16 点 DFT 第 0 频点
void utils_fft16_bin1(float *real_in, float *real, float *imag);                                               // 16 点 DFT 第 1 频点
void utils_fft16_bin2(float *real_in, float *real, float *imag);                                               // 16 点 DFT 第 2 频点
void utils_fft8_bin0(float *real_in, float *real, float *imag);                                                // 8 点 DFT 第 0 频点
void utils_fft8_bin1(float *real_in, float *real, float *imag);                                                // 8 点 DFT 第 1 频点
void utils_fft8_bin2(float *real_in, float *real, float *imag);                                                // 8 点 DFT 第 2 频点
float utils_batt_liion_norm_v_to_capacity(float norm_v);                                                       // 锂电电压->容量
uint16_t utils_median_filter_uint16_run(uint16_t *buffer,
                                        unsigned int *buffer_index, unsigned int filter_len, uint16_t sample); // 中值滤波
void utils_rotate_vector3(float *input, float *rotation, float *output, bool reverse);                         // 三维向量旋转

/* 数学常量（双精度定义，单精度场景由调用方强转） */
#define M_E        2.71828182845904523536  // e
#define M_LOG2E    1.44269504088896340736  // log2(e)
#define M_LOG10E   0.434294481903251827651 // log10(e)
#define M_LN2      0.693147180559945309417 // ln(2)
#define M_LN10     2.30258509299404568402  // ln(10)
#define M_PI       3.14159265358979323846  // 圆周率 pi
#define M_PI_2     1.57079632679489661923  // pi/2
#define M_PI_4     0.785398163397448309616 // pi/4
#define M_1_PI     0.318309886183790671538 // 1/pi
#define M_2_PI     0.636619772367581343076 // 2/pi
#define M_2_SQRTPI 1.12837916709551257390  // 2/sqrt(pi)
#define M_SQRT2    1.41421356237309504880  // sqrt(2)
#define M_SQRT1_2  0.707106781186547524401 // 1/sqrt(2)

/* 取符号：负数返回 -1，零或正数返回 +1 */
#define SIGN(x) (((x) < 0.0) ? -1.0 : 1.0)

/* 平方 */
#define SQ(x) ((x) * (x))

/* 二维向量的二范数（单精度） */
//#define NORM2(x,y)		(sqrt(SQ(x) + SQ(y)))
#define NORM2_f(x, y) (sqrtf(SQ(x) + SQ(y)))

/* 浮点 NaN / 无穷判断与置零 */
#define UTILS_IS_INF(x)   ((x) == (1.0 / 0.0) || (x) == (-1.0 / 0.0))
#define UTILS_IS_NAN(x)   ((x) != (x))
#define UTILS_NAN_ZERO(x) (x = UTILS_IS_NAN(x) ? 0.0 : x)

/* 角度/弧度、RPM/(弧度每秒) 单精度互转（系数为编译期常量表达式） */
#define DEG2RAD_f(deg)           ((deg) * (float)(M_PI / 180.0))
#define RAD2DEG_f(rad)           ((rad) * (float)(180.0 / M_PI))
#define RPM2RADPS_f(rpm)         ((rpm) * (float)((2.0 * M_PI) / 60.0))
#define RADPS2RPM_f(rad_per_sec) ((rad_per_sec) * (float)(60.0 / (2.0 * M_PI)))

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

/* 双精度字面量构造辅助 */
#define D(x) ((double)x##L)

/**
 * @brief   一阶低通滤波（IIR）
 * @param   value           [in,out] 被滤波值
 * @param   sample          新采样
 * @param   filter_constant 滤波系数，范围 0.0~1.0，越接近 1.0 越接近原始值
 */
#define UTILS_LP_FAST(value, sample, filter_constant) (value -= (filter_constant) * ((value) - (sample)))

/**
 * @brief   N 点移动平均滤波的快速近似
 * @details 参见 https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
 *          与 https://en.wikipedia.org/wiki/Exponential_smoothing
 *          行为更接近 IIR 而非 FIR，但占用内存极小、运行更快。
 * @param   value   [in,out] 被滤波值
 * @param   sample  新采样
 * @param   N       等效样本数
 */
#define UTILS_LP_MOVING_AVG_APPROX(value, sample, N) UTILS_LP_FAST(value, sample, 2.0 / ((N) + 1.0))

/* 常量（单精度） */
#define ONE_BY_SQRT3     (0.57735026919f)
#define TWO_BY_SQRT3     (2.0f * 0.57735026919f)
#define SQRT3_BY_2       (0.86602540378f)
#define COS_30_DEG       (0.86602540378f)
#define SIN_30_DEG       (0.5f)
#define COS_MINUS_30_DEG (0.86602540378f)
#define SIN_MINUS_30_DEG (-0.5f)
#define ONE_BY_SQRT2     (0.7071067811865475f)

/* DFT 旋转因子查找表（定义见 utils.c） */
extern const float utils_tab_sin_32_1[];
extern const float utils_tab_sin_32_2[];
extern const float utils_tab_cos_32_1[];
extern const float utils_tab_cos_32_2[];

/* 内联函数 */
/**
 * @brief   以固定步长将 value 朝 goal 逼近，不会越过目标
 * @param   value   [in,out] 当前值的指针
 * @param   goal    目标值
 * @param   step    每次逼近步长（正值）
 */
static inline void utils_step_towards(float *value, float goal, float step)
{
	if (*value < goal)
	{
		if ((*value + step) < goal)
		{
			*value += step;
		}
		else
		{
			*value = goal;
		}
	}
	else if (*value > goal)
	{
		if ((*value - step) > goal)
		{
			*value -= step;
		}
		else
		{
			*value = goal;
		}
	}
}

/**
 * @brief   将角度归一化到 [0, 360) 度
 * @param   angle   [in,out] 待归一化角度的指针（度）
 */
static inline void utils_norm_angle(float *angle)
{
	*angle = fmodf(*angle, 360.0f);

	if (*angle < 0.0f)
	{
		*angle += 360.0f;
	}
}

/**
 * @brief   将角度归一化到 [-pi, pi) 弧度
 * @param   angle   [in,out] 待归一化角度的指针（弧度）
 * @warning 勿传入过大角度（循环逐步逼近，过大将耗时）
 */
static inline void utils_norm_angle_rad(float *angle)
{
	while (*angle < -(float)M_PI)
	{
		*angle += 2.0f * (float)M_PI;
	}
	while (*angle >= (float)M_PI)
	{
		*angle -= 2.0f * (float)M_PI;
	}
}

/**
 * @brief   将浮点数限幅到 [min, max]
 * @param   number  [in,out] 待限幅值的指针
 * @param   min     下限
 * @param   max     上限
 */
static inline void utils_truncate_number(float *number, float min, float max)
{
	if (*number > max)
	{
		*number = max;
	}
	else if (*number < min)
	{
		*number = min;
	}
}

/**
 * @brief   将整数限幅到 [min, max]
 * @param   number  [in,out] 待限幅值的指针
 * @param   min     下限
 * @param   max     上限
 */
static inline void utils_truncate_number_int(int *number, int min, int max)
{
	if (*number > max)
	{
		*number = max;
	}
	else if (*number < min)
	{
		*number = min;
	}
}

/**
 * @brief   将浮点数按绝对值限幅到 [-max, max]
 * @param   number  [in,out] 待限幅值的指针
 * @param   max     幅值上限
 */
static inline void utils_truncate_number_abs(float *number, float max)
{
	if (*number > max)
	{
		*number = max;
	}
	else if (*number < -max)
	{
		*number = -max;
	}
}

/**
 * @brief   线性映射：将 x 从 [in_min,in_max] 映射到 [out_min,out_max]
 */
static inline float utils_map(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief   整数线性映射：将 x 从 [in_min,in_max] 映射到 [out_min,out_max]
 */
static inline int utils_map_int(int x, int in_min, int in_max, int out_min, int out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief   对二维向量的幅值限幅
 * @param   x       [in,out] 第一个分量的指针
 * @param   y       [in,out] 第二个分量的指针
 * @param   max     允许的最大幅值
 * @return  发生饱和返回 true，否则 false
 */
static inline bool utils_saturate_vector_2d(float *x, float *y, float max)
{
	bool retval = false;
	float mag = NORM2_f(*x, *y);
	max = fabsf(max);

	if (mag < 1e-10f)
	{
		mag = 1e-10f;
	}

	if (mag > max)
	{
		const float f = max / mag;
		*x *= f;
		*y *= f;
		retval = true;
	}

	return retval;
}

#endif /* UTILS_MATH_H_ */
