/*
 * main.c
 *
 *  Created on: 2018年7月9日
 *      Author: 陆相如
 */

#include "stm32f10x.h"
#include "delay.h"
#include "xprintf.h"
#include "bsp_can.h"
// #include "bsp_timer.h"
#include "bsp_rs485_usart.h"
#include "can_usart_convert_protocol.h"
#include "ring_queue.h"

int main(void)
{
	CanRxMsg value;
//	unsigned char ledCnt=0;
	init_queue();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	// BSP_SYSTIME_INTR_Init(999,360-1);    //100kHz 计数1000 10ms产生一次溢出中断
	
	BSP_CAN_Config(CAN_SPEED_500K);
	// CAN1_Init();
	USART2_Initialise(1046400);
	for (;;)
	{
		xprintf("rs485Data Waiting");
		
//		ledCnt++;					//chrade add debug purpose. 191113
//		if(ledCnt >20){
//		//	BSP_CAN_SetMsg(0x01, 0x02);
//		//	USART2_SendData("1234", 4);
//			ledCnt=0;
//		}			
		while(out_queue(&value) == true)
		{
			deal_can_ext_frame(&value);
		}
		Delay_ms(10);
	}
	BSP_CAN_DeInit();
}
