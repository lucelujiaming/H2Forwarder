#ifndef __BSP_CAN_H
#define	__BSP_CAN_H

#include "stm32f10x.h"
#include <stdio.h>


//pin definition shouldn't define at here. 
//which should defined in: ../../Boards/inc/board_XXXX.h
/*debug*/

void BSP_CAN_Config(uint8_t bitRate);
void BSP_CAN_DeInit(void);

u8 BSP_CAN_SetMsg(u8 Data1,u8 Data2);
#endif







