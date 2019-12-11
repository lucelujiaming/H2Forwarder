/*
 * RS-485.h
 *
 *  Created on: 2019年11月8日
 *      Author: 陆相如
 */

#ifndef HAL_RS485_H_
#define HAL_RS485_H_

#include "stm32f10x.h"
// #include "niu_type.h"

#ifdef __cplusplus
extern "C" {
#endif

void RS485_Init(uint32_t baudrate);
void RS485_Send_Data(u8 *buf,u8 len) ;


#ifdef __cplusplus
{
#endif

#endif /* HAL_RS485_H_ */
