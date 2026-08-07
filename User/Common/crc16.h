/**
 * @file crc16.h
 * @brief CRC16（CCITT）校验与累加校验和接口
 * @author Dalin
 * @version 1.00
 * @date 2024-11-13
 * 
 * @copyright Copyright (c) 2024  RobotDance Technology Co., Ltd.
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date           <th>Version     <th>Author      <th>Description
 * <tr><td>2024-11-13     <td>1.00        <td>LinHui      <td>Init
 * </table>
 */

#ifndef _CRC_CRC16_H
#define _CRC_CRC16_H

#include <stdint.h>
#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus  */

	uint16_t crc16_calc(uint8_t *buf, int len);
	uint16_t check_sum(uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif /* __cplusplus  */
#endif
