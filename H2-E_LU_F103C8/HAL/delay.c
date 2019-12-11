#include "delay.h"

extern u8 OS_RUNNING;

void Delay_ms(volatile u16 nCount)
{
	volatile u32 Count;
#if defined(FreeRTOS)
	if(OS_RUNNING)
	{
		vTaskDelay( nCount/portTICK_RATE_MS);
	}
	else
	{
#endif	
		Count=nCount*10000;
		for(; Count != 0; Count--)
		 {
					 __NOP();
					 __NOP();
					 __NOP();
					 __NOP();
		 
		 }
#if defined(FreeRTOS)
	}
#endif
}

void Delay_us(volatile u32 time)
{
  volatile u32 i=8*time;
  while(i--);
}

#if 0
void delay_us(u32 time)
{
  u32 i=8*time;
  while(i--);
}
#endif
