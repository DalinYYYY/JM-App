/**
 * @file        utils.c
 * @brief       通用数学与信号处理工具集（角度归一化、快速三角函数、FFT、滤波等）
 *
 * @details     本模块面向 Cortex-M4F 单精度 FPU 优化：所有浮点字面量均带 f 后缀，
 *              避免单精度操作数被隐式提升为双精度（编译告警 #1035-D），从而保证
 *              运算全程走单精度硬件 FPU，消除 double 软/硬件转换开销。
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-06-17
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#include "utils.h"

#include <string.h>
#include <stdlib.h>

/**
 * @brief   将角度映射到 [0,1] 区间（对应 [min,max]）
 * @details 若角度超出范围，则截断到最近的边界。角度单位：度。
 * @param   angle   待映射角度（度）
 * @param   min     范围下限（度）
 * @param   max     范围上限（度）
 * @return  映射后的 [0,1] 值；min==max 时返回 -1
 */
float utils_map_angle(float angle, float min, float max)
{
	if (max == min)
	{
		return -1;
	}

	float range_pos = max - min;
	utils_norm_angle(&range_pos);
	float range_neg = min - max;
	utils_norm_angle(&range_neg);
	float margin = range_neg / 2.0f;

	angle -= min;
	utils_norm_angle(&angle);
	if (angle > (360 - margin))
	{
		angle -= 360.0f;
	}

	float res = angle / range_pos;
	utils_truncate_number(&res, 0.0f, 1.0f);

	return res;
}

/**
 * @brief   死区处理：绝对值小于 tres 的输入截断为 0
 * @details tres 映射为 0，max 映射为 max，区间内线性过渡。
 * @param   value   [in,out] 待处理值的指针
 * @param   tres    死区阈值
 * @param   max     满量程值
 */
void utils_deadband(float *value, float tres, float max)
{
	if (fabsf(*value) < tres)
	{
		*value = 0.0f;
	}
	else
	{
		float k = max / (max - tres);
		if (*value > 0.0f)
		{
			*value = k * *value + max * (1.0f - k);
		}
		else
		{
			*value = -(k * -*value + max * (1.0f - k));
		}
	}
}

/**
 * @brief   计算两个角度之差，结果恒在 [-180, +180] 度
 * @param   angle1  第一个角度（度）
 * @param   angle2  第二个角度（度）
 * @return  角度差（度）
 */
float utils_angle_difference(float angle1, float angle2)
{
	float difference = angle1 - angle2;
	while (difference < -180.0f)
		difference += 2.0f * 180.0f;
	while (difference > 180.0f)
		difference -= 2.0f * 180.0f;
	return difference;
}

/**
 * @brief   计算两个角度之差，结果恒在 [-pi, +pi] 弧度
 * @param   angle1  第一个角度（弧度）
 * @param   angle2  第二个角度（弧度）
 * @return  角度差（弧度）
 */
float utils_angle_difference_rad(float angle1, float angle2)
{
	float difference = angle1 - angle2;
	while (difference < -(float)M_PI)
		difference += 2.0f * (float)M_PI;
	while (difference > (float)M_PI)
		difference -= 2.0f * (float)M_PI;
	return difference;
}

/**
 * @brief   求一组角度的加权平均（弧度）
 * @details 通过对各角度的 sin/cos 加权求和后取 atan2，避免角度环绕问题。
 * @param   angles      角度数组（弧度）
 * @param   weights     各角度对应权重
 * @param   angles_num  角度个数
 * @return  平均角度（弧度）
 */
float utils_avg_angles_rad_fast(float *angles, float *weights, int angles_num)
{
	float s_sum = 0.0f;
	float c_sum = 0.0f;

	for (int i = 0; i < angles_num; i++)
	{
		float s, c;
		utils_fast_sincos_better(angles[i], &s, &c);
		s_sum += s * weights[i];
		c_sum += c * weights[i];
	}

	return utils_fast_atan2(s_sum, c_sum);
}

/**
 * @brief   取三个浮点值的中间值
 * @param   a   第一个值
 * @param   b   第二个值
 * @param   c   第三个值
 * @return  中间值
 */
float utils_middle_of_3(float a, float b, float c)
{
	float middle;

	if ((a <= b) && (a <= c))
	{
		middle = (b <= c) ? b : c;
	}
	else if ((b <= a) && (b <= c))
	{
		middle = (a <= c) ? a : c;
	}
	else
	{
		middle = (a <= b) ? a : b;
	}
	return middle;
}

/**
 * @brief   取三个整数值的中间值
 * @param   a   第一个值
 * @param   b   第二个值
 * @param   c   第三个值
 * @return  中间值
 */
int utils_middle_of_3_int(int a, int b, int c)
{
	int middle;

	if ((a <= b) && (a <= c))
	{
		middle = (b <= c) ? b : c;
	}
	else if ((b <= a) && (b <= c))
	{
		middle = (a <= c) ? a : c;
	}
	else
	{
		middle = (a <= b) ? a : b;
	}
	return middle;
}

/**
 * @brief   快速 atan2 近似实现
 * @details 参考 http://www.dspguru.com/dsp/tricks/fixed-point-atan2-with-self-normalization
 * @param   y   y 分量
 * @param   x   x 分量
 * @return  角度（弧度）
 */
float utils_fast_atan2(float y, float x)
{
	float abs_y = fabsf(y) + 1e-20f; // 防止 0/0 的微小偏置
	float angle;

	if (x >= 0)
	{
		float r = (x - abs_y) / (x + abs_y);
		float rsq = r * r;
		angle = ((0.1963f * rsq) - 0.9817f) * r + (float)(M_PI / 4.0);
	}
	else
	{
		float r = (x + abs_y) / (abs_y - x);
		float rsq = r * r;
		angle = ((0.1963f * rsq) - 0.9817f) * r + (float)(3.0 * M_PI / 4.0);
	}

	UTILS_NAN_ZERO(angle);

	if (y < 0)
	{
		return (-angle);
	}
	else
	{
		return (angle);
	}
}

/**
 * @brief   快速 sin/cos 近似实现
 * @details 参考 http://lab.polygonal.de/?p=205
 * @param   angle   角度（弧度），警告：勿传入过大角度
 * @param   sin     [out] 存储正弦值的指针
 * @param   cos     [out] 存储余弦值的指针
 */
void utils_fast_sincos(float angle, float *sin, float *cos)
{
	// 始终将输入角度环绕到 -PI..PI
	while (angle < -(float)M_PI)
	{
		angle += 2.0f * (float)M_PI;
	}

	while (angle > (float)M_PI)
	{
		angle -= 2.0f * (float)M_PI;
	}

	// 计算正弦
	if (angle < 0.0f)
	{
		*sin = 1.27323954f * angle + 0.405284735f * angle * angle;
	}
	else
	{
		*sin = 1.27323954f * angle - 0.405284735f * angle * angle;
	}

	// 计算余弦：sin(x + PI/2) = cos(x)
	angle += 0.5f * (float)M_PI;

	if (angle > (float)M_PI)
	{
		angle -= 2.0f * (float)M_PI;
	}

	if (angle < 0.0f)
	{
		*cos = 1.27323954f * angle + 0.405284735f * angle * angle;
	}
	else
	{
		*cos = 1.27323954f * angle - 0.405284735f * angle * angle;
	}
}

/**
 * @brief   更高精度的快速 sin/cos 近似实现
 * @details 参考 http://lab.polygonal.de/?p=205 ，在基础近似上叠加一次修正项以提升精度。
 * @param   angle   角度（弧度），警告：勿传入过大角度
 * @param   sin     [out] 存储正弦值的指针
 * @param   cos     [out] 存储余弦值的指针
 */
void utils_fast_sincos_better(float angle, float *sin, float *cos)
{
	// 始终将输入角度环绕到 -PI..PI
	while (angle < -(float)M_PI)
	{
		angle += 2.0f * (float)M_PI;
	}

	while (angle > (float)M_PI)
	{
		angle -= 2.0f * (float)M_PI;
	}

	// 计算正弦
	if (angle < 0.0f)
	{
		*sin = 1.27323954f * angle + 0.405284735f * angle * angle;

		if (*sin < 0.0f)
		{
			*sin = 0.225f * (*sin * -*sin - *sin) + *sin;
		}
		else
		{
			*sin = 0.225f * (*sin * *sin - *sin) + *sin;
		}
	}
	else
	{
		*sin = 1.27323954f * angle - 0.405284735f * angle * angle;

		if (*sin < 0.0f)
		{
			*sin = 0.225f * (*sin * -*sin - *sin) + *sin;
		}
		else
		{
			*sin = 0.225f * (*sin * *sin - *sin) + *sin;
		}
	}

	// 计算余弦：sin(x + PI/2) = cos(x)
	angle += 0.5f * (float)M_PI;
	if (angle > (float)M_PI)
	{
		angle -= 2.0f * (float)M_PI;
	}

	if (angle < 0.0f)
	{
		*cos = 1.27323954f * angle + 0.405284735f * angle * angle;

		if (*cos < 0.0f)
		{
			*cos = 0.225f * (*cos * -*cos - *cos) + *cos;
		}
		else
		{
			*cos = 0.225f * (*cos * *cos - *cos) + *cos;
		}
	}
	else
	{
		*cos = 1.27323954f * angle - 0.405284735f * angle * angle;

		if (*cos < 0.0f)
		{
			*cos = 0.225f * (*cos * -*cos - *cos) + *cos;
		}
		else
		{
			*cos = 0.225f * (*cos * *cos - *cos) + *cos;
		}
	}
}

/**
 * @brief   返回幅值（绝对值）较小的那个值
 * @param   va  第一个值
 * @param   vb  第二个值
 * @return  幅值较小的值
 */
float utils_min_abs(float va, float vb)
{
	float res;
	if (fabsf(va) < fabsf(vb))
	{
		res = va;
	}
	else
	{
		res = vb;
	}

	return res;
}

/**
 * @brief   返回幅值（绝对值）较大的那个值
 * @param   va  第一个值
 * @param   vb  第二个值
 * @return  幅值较大的值
 */
float utils_max_abs(float va, float vb)
{
	float res;
	if (fabsf(va) > fabsf(vb))
	{
		res = va;
	}
	else
	{
		res = vb;
	}

	return res;
}

/**
 * @brief   将一个字节的二进制内容生成字符串表示
 * @param   x   字节值
 * @param   b   [out] 存储字符串表示的数组（至少 9 字节）
 */
void utils_byte_to_binary(int x, char *b)
{
	b[0] = '\0';

	int z;
	for (z = 128; z > 0; z >>= 1)
	{
		strcat(b, ((x & z) == z) ? "1" : "0");
	}
}

/**
 * @brief   油门/控制量曲线整形
 * @details 将 [-1,1] 输入按指定曲线模式整形输出，正负方向可分别设定曲率。
 * @param   val         输入值，范围 [-1,1]（超出自动截断）
 * @param   curve_acc   正向（加速）曲率
 * @param   curve_brake 负向（刹车）曲率
 * @param   mode        曲线模式：0 指数 / 1 自然指数 / 2 多项式 / 其他 线性
 * @return  整形后的输出值
 */
float utils_throttle_curve(float val, float curve_acc, float curve_brake, int mode)
{
	float ret = 0.0f;

	if (val < -1.0f)
	{
		val = -1.0f;
	}

	if (val > 1.0f)
	{
		val = 1.0f;
	}

	float val_a = fabsf(val);

	float curve;
	if (val >= 0.0f)
	{
		curve = curve_acc;
	}
	else
	{
		curve = curve_brake;
	}

	// 曲线公式参考 http://math.stackexchange.com/questions/297768
	if (mode == 0)
	{ // 指数
		if (curve >= 0.0f)
		{
			ret = 1.0f - powf(1.0f - val_a, 1.0f + curve);
		}
		else
		{
			ret = powf(val_a, 1.0f - curve);
		}
	}
	else if (mode == 1)
	{ // 自然指数
		if (fabsf(curve) < 1e-10f)
		{
			ret = val_a;
		}
		else
		{
			if (curve >= 0.0f)
			{
				ret = 1.0f - ((expf(curve * (1.0f - val_a)) - 1.0f) / (expf(curve) - 1.0f));
			}
			else
			{
				ret = (expf(-curve * val_a) - 1.0f) / (expf(-curve) - 1.0f);
			}
		}
	}
	else if (mode == 2)
	{ // 多项式
		if (curve >= 0.0f)
		{
			ret = 1.0f - ((1.0f - val_a) / (1.0f + curve * val_a));
		}
		else
		{
			ret = val_a / (1.0f - curve * (1.0f - val_a));
		}
	}
	else
	{ // 线性
		ret = val_a;
	}

	if (val < 0.0f)
	{
		ret = -ret;
	}

	return ret;
}

/**
 * @brief   计算 CRC32C（Castagnoli 多项式 0x82F63B78）校验值
 * @param   data    数据缓冲区
 * @param   len     数据长度（字节）
 * @return  CRC32C 校验值
 */
uint32_t utils_crc32c(uint8_t *data, uint32_t len)
{
	uint32_t crc = 0xFFFFFFFF;

	for (uint32_t i = 0; i < len; i++)
	{
		uint32_t byte = data[i];
		crc = crc ^ byte;

		for (int j = 7; j >= 0; j--)
		{
			uint32_t mask = -(crc & 1);
			crc = (crc >> 1) ^ (0x82F63B78 & mask);
		}
	}

	return ~crc;
}

/**
 * @brief   32 点 DFT 的第 0 频点（直流分量，即均值）
 * @param   real_in 输入实数序列（32 点）
 * @param   real    [out] 实部
 * @param   imag    [out] 虚部（恒为 0）
 */
void utils_fft32_bin0(float *real_in, float *real, float *imag)
{
	*real = 0.0f;
	*imag = 0.0f;

	for (int i = 0; i < 32; i++)
	{
		*real += real_in[i];
	}

	*real /= 32.0f;
}

/**
 * @brief   32 点 DFT 的第 1 频点
 * @param   real_in 输入实数序列（32 点）
 * @param   real    [out] 实部
 * @param   imag    [out] 虚部
 */
void utils_fft32_bin1(float *real_in, float *real, float *imag)
{
	*real = 0.0f;
	*imag = 0.0f;
	for (int i = 0; i < 32; i++)
	{
		*real += real_in[i] * utils_tab_cos_32_1[i];
		*imag -= real_in[i] * utils_tab_sin_32_1[i];
	}
	*real /= 32.0f;
	*imag /= 32.0f;
}

/**
 * @brief   32 点 DFT 的第 2 频点
 * @param   real_in 输入实数序列（32 点）
 * @param   real    [out] 实部
 * @param   imag    [out] 虚部
 */
void utils_fft32_bin2(float *real_in, float *real, float *imag)
{
	*real = 0.0f;
	*imag = 0.0f;
	for (int i = 0; i < 32; i++)
	{
		*real += real_in[i] * utils_tab_cos_32_2[i];
		*imag -= real_in[i] * utils_tab_sin_32_2[i];
	}
	*real /= 32.0f;
	*imag /= 32.0f;
}

/**
 * @brief   16 点 DFT 的第 0 频点（直流分量，即均值）
 * @param   real_in 输入实数序列（16 点）
 * @param   real    [out] 实部
 * @param   imag    [out] 虚部（恒为 0）
 */
void utils_fft16_bin0(float *real_in, float *real, float *imag)
{
	*real = 0.0f;
	*imag = 0.0f;

	for (int i = 0; i < 16; i++)
	{
		*real += real_in[i];
	}

	*real /= 16.0f;
}

/**
 * @brief   16 点 DFT 的第 1 频点
 * @param   real_in 输入实数序列（16 点）
 * @param   real    [out] 实部
 * @param   imag    [out] 虚部
 */
void utils_fft16_bin1(float *real_in, float *real, float *imag)
{
	*real = 0.0f;
	*imag = 0.0f;
	for (int i = 0; i < 16; i++)
	{
		*real += real_in[i] * utils_tab_cos_32_1[2 * i];
		*imag -= real_in[i] * utils_tab_sin_32_1[2 * i];
	}
	*real /= 16.0f;
	*imag /= 16.0f;
}

/**
 * @brief   16 点 DFT 的第 2 频点
 * @param   real_in 输入实数序列（16 点）
 * @param   real    [out] 实部
 * @param   imag    [out] 虚部
 */
void utils_fft16_bin2(float *real_in, float *real, float *imag)
{
	*real = 0.0f;
	*imag = 0.0f;
	for (int i = 0; i < 16; i++)
	{
		*real += real_in[i] * utils_tab_cos_32_2[2 * i];
		*imag -= real_in[i] * utils_tab_sin_32_2[2 * i];
	}
	*real /= 16.0f;
	*imag /= 16.0f;
}

/**
 * @brief   8 点 DFT 的第 0 频点（直流分量，即均值）
 * @param   real_in 输入实数序列（8 点）
 * @param   real    [out] 实部
 * @param   imag    [out] 虚部（恒为 0）
 */
void utils_fft8_bin0(float *real_in, float *real, float *imag)
{
	*real = 0.0f;
	*imag = 0.0f;

	for (int i = 0; i < 8; i++)
	{
		*real += real_in[i];
	}

	*real /= 8.0f;
}

/**
 * @brief   8 点 DFT 的第 1 频点
 * @param   real_in 输入实数序列（8 点）
 * @param   real    [out] 实部
 * @param   imag    [out] 虚部
 */
void utils_fft8_bin1(float *real_in, float *real, float *imag)
{
	*real = 0.0f;
	*imag = 0.0f;
	for (int i = 0; i < 8; i++)
	{
		*real += real_in[i] * utils_tab_cos_32_1[4 * i];
		*imag -= real_in[i] * utils_tab_sin_32_1[4 * i];
	}
	*real /= 8.0f;
	*imag /= 8.0f;
}

/**
 * @brief   8 点 DFT 的第 2 频点
 * @param   real_in 输入实数序列（8 点）
 * @param   real    [out] 实部
 * @param   imag    [out] 虚部
 */
void utils_fft8_bin2(float *real_in, float *real, float *imag)
{
	*real = 0.0f;
	*imag = 0.0f;
	for (int i = 0; i < 8; i++)
	{
		*real += real_in[i] * utils_tab_cos_32_2[4 * i];
		*imag -= real_in[i] * utils_tab_sin_32_2[4 * i];
	}
	*real /= 8.0f;
	*imag /= 8.0f;
}

/**
 * @brief   三星 30Q 锂电芯归一化电压 -> 剩余容量百分比映射
 * @details 电压范围 4.2V~3.2V，注意该区间内会损失额定 3Ah 容量的约 15%。
 *          采用 5 阶多项式拟合。
 * @param   norm_v  归一化电压（0~1，自动截断）
 * @return  剩余容量（归一化）
 */
float utils_batt_liion_norm_v_to_capacity(float norm_v)
{
	// 锂离子电池多项式拟合系数
	const float li_p[] = {
		-2.979767f, 5.487810f, -3.501286f, 1.675683f, 0.317147f};
	utils_truncate_number(&norm_v, 0.0f, 1.0f);
	float v2 = norm_v * norm_v;
	float v3 = v2 * norm_v;
	float v4 = v3 * norm_v;
	float v5 = v4 * norm_v;
	float capacity = li_p[0] * v5 + li_p[1] * v4 + li_p[2] * v3 + li_p[3] * v2 + li_p[4] * norm_v;
	return capacity;
}

/* uint16 升序比较函数，供 qsort 使用 */
static int uint16_cmp_func(const void *a, const void *b)
{
	return (*(uint16_t *)a - *(uint16_t *)b);
}

/**
 * @brief   uint16 中值滤波器单步运行
 * @details 将新采样写入环形缓冲区，对副本排序后返回中位数。
 * @param   buffer          环形缓冲区
 * @param   buffer_index    [in,out] 当前写入索引
 * @param   filter_len      滤波窗口长度
 * @param   sample          本次新采样
 * @return  当前窗口的中值
 */
uint16_t utils_median_filter_uint16_run(uint16_t *buffer,
                                        unsigned int *buffer_index, unsigned int filter_len, uint16_t sample)
{
	buffer[(*buffer_index)++] = sample;
	*buffer_index %= filter_len;
	uint16_t buffer_sorted[filter_len]; // 假定栈空间足够
	memcpy(buffer_sorted, buffer, sizeof(uint16_t) * filter_len);
	qsort(buffer_sorted, filter_len, sizeof(uint16_t), uint16_cmp_func);
	return buffer_sorted[filter_len / 2];
}

/**
 * @brief   对三维向量做 ZYX 欧拉角旋转
 * @param   input       输入向量 [x,y,z]
 * @param   rotation    旋转角 [roll, pitch, yaw]（弧度，索引 0/1/2）
 * @param   output      [out] 旋转后向量
 * @param   reverse     true 为逆向旋转（转置矩阵）
 */
void utils_rotate_vector3(float *input, float *rotation, float *output, bool reverse)
{
	float s1, c1, s2, c2, s3, c3;

	if (rotation[2] != 0.0f)
	{
		s1 = sinf(rotation[2]);
		c1 = cosf(rotation[2]);
	}
	else
	{
		s1 = 0.0f;
		c1 = 1.0f;
	}

	if (rotation[1] != 0.0f)
	{
		s2 = sinf(rotation[1]);
		c2 = cosf(rotation[1]);
	}
	else
	{
		s2 = 0.0f;
		c2 = 1.0f;
	}

	if (rotation[0] != 0.0f)
	{
		s3 = sinf(rotation[0]);
		c3 = cosf(rotation[0]);
	}
	else
	{
		s3 = 0.0f;
		c3 = 1.0f;
	}

	float m11 = c1 * c2;
	float m12 = c1 * s2 * s3 - c3 * s1;
	float m13 = s1 * s3 + c1 * c3 * s2;
	float m21 = c2 * s1;
	float m22 = c1 * c3 + s1 * s2 * s3;
	float m23 = c3 * s1 * s2 - c1 * s3;
	float m31 = -s2;
	float m32 = c2 * s3;
	float m33 = c2 * c3;

	if (reverse)
	{
		output[0] = input[0] * m11 + input[1] * m21 + input[2] * m31;
		output[1] = input[0] * m12 + input[1] * m22 + input[2] * m32;
		output[2] = input[0] * m13 + input[1] * m23 + input[2] * m33;
	}
	else
	{
		output[0] = input[0] * m11 + input[1] * m12 + input[2] * m13;
		output[1] = input[0] * m21 + input[1] * m22 + input[2] * m23;
		output[2] = input[0] * m31 + input[1] * m32 + input[2] * m33;
	}
}

/* DFT 旋转因子查找表：32 点 sin/cos，下标 1/2 分别对应基波与二次谐波 */
const float utils_tab_sin_32_1[] = {
	0.000000, 0.195090, 0.382683, 0.555570, 0.707107, 0.831470, 0.923880, 0.980785,
	1.000000, 0.980785, 0.923880, 0.831470, 0.707107, 0.555570, 0.382683, 0.195090,
	0.000000, -0.195090, -0.382683, -0.555570, -0.707107, -0.831470, -0.923880, -0.980785,
	-1.000000, -0.980785, -0.923880, -0.831470, -0.707107, -0.555570, -0.382683, -0.195090};

const float utils_tab_sin_32_2[] = {
	0.000000, 0.382683, 0.707107, 0.923880, 1.000000, 0.923880, 0.707107, 0.382683,
	0.000000, -0.382683, -0.707107, -0.923880, -1.000000, -0.923880, -0.707107, -0.382683,
	-0.000000, 0.382683, 0.707107, 0.923880, 1.000000, 0.923880, 0.707107, 0.382683,
	0.000000, -0.382683, -0.707107, -0.923880, -1.000000, -0.923880, -0.707107, -0.382683};

const float utils_tab_cos_32_1[] = {
	1.000000, 0.980785, 0.923880, 0.831470, 0.707107, 0.555570, 0.382683, 0.195090,
	0.000000, -0.195090, -0.382683, -0.555570, -0.707107, -0.831470, -0.923880, -0.980785,
	-1.000000, -0.980785, -0.923880, -0.831470, -0.707107, -0.555570, -0.382683, -0.195090,
	-0.000000, 0.195090, 0.382683, 0.555570, 0.707107, 0.831470, 0.923880, 0.980785};

const float utils_tab_cos_32_2[] = {
	1.000000, 0.923880, 0.707107, 0.382683, 0.000000, -0.382683, -0.707107, -0.923880,
	-1.000000, -0.923880, -0.707107, -0.382683, -0.000000, 0.382683, 0.707107, 0.923880,
	1.000000, 0.923880, 0.707107, 0.382683, 0.000000, -0.382683, -0.707107, -0.923880,
	-1.000000, -0.923880, -0.707107, -0.382683, -0.000000, 0.382683, 0.707107, 0.923880};
