#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H
//#include "sys.h"
/* Includes ------------------------------------------------------------------*/

void BSP_SYSTIME_Init(void);
uint32_t micros(void);
uint32_t millis(void);	 	 
//Get delta Time
uint32_t get_delta_time(uint32_t lastTime);

void BSP_SYSTIME_INTR_Init(u16 arr,u16 psc);

#endif

