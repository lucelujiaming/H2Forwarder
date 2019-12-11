/*
 * delay.h
 *
 *  Created on: 2017年10月25日
 *      Author: 陆相如
 */

#ifndef HAL_DELAY_H_
#define HAL_DELAY_H_

#include "stm32f10x.h"
#if defined(FreeRTOS)
#include "FreeRTOS.h"
#include "task.h"
#endif

void Delay_us(volatile u32 time);
void Delay_ms(volatile u16 nCount);


#endif /* HAL_DELAY_H_ */
