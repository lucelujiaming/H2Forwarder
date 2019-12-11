
#define BSP_TIMER
/*******************************************************/
#define BASE_TIMER          TIM2
#define BASE_TIMER_CLOCK    RCC_APB1Periph_TIM2
#define BASE_TIMER_MAX      0xFFFF   //0xFFFFFFFF

#define BASE_TIMER_INTR          TIM4
#define BASE_TIMER_INTR_CLOCK    RCC_APB1Periph_TIM4
#define BASE_TIMER_INTR_IRQ         TIM4_IRQn

#define BASE_TIMER_INTR_IRQHandler			TIM4_IRQHandler
/*******************************************************/

#ifdef BSP_TIMER
#include "stm32f10x.h"
#include "bsp_timer.h"
#include "ring_queue.h"
#include "can_usart_convert_protocol.h"
//pin definition shouldn't define at here. 
//which should defined in: ../../Boards/inc/board_XXXX.h

void BSP_SYSTIME_Init(void)//void BSP_TIMER_Init(void)
{   
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;	
	RCC_APB1PeriphClockCmd(BASE_TIMER_CLOCK,ENABLE);               //enable timer2 rcc clock    

	TIM_TimeBaseInitStructure.TIM_Period = BASE_TIMER_MAX; 	       //reloader value ---> 2^32/10^6/3600 = 1.193h = 71min
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;              //1Mhz ---> 1 tick = 1us
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;  
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 	
	TIM_TimeBaseInit(BASE_TIMER,&TIM_TimeBaseInitStructure);       //init timer2
							 
	TIM_Cmd(BASE_TIMER,ENABLE);                                    //cmd timer2
	
}


uint32_t micros(void)
{
  return BASE_TIMER->CNT;
}
 
uint32_t millis(void) 
{
  return (BASE_TIMER->CNT / 1000);
}

uint32_t get_delta_time(uint32_t lastTime)
{
	uint32_t timeNow = micros();
    
	if(timeNow > lastTime)
	{
		//overflow, devo correggere	
		return (timeNow - lastTime);	
	}
    
  return (timeNow + (1 + BASE_TIMER_MAX - lastTime));    	
}



void BSP_SYSTIME_INTR_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(BASE_TIMER_INTR_CLOCK, ENABLE); //??TIM3??
	
	TIM_TimeBaseStructure.TIM_Period = arr; 					//??????
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 					//????
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 	//??????
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//??????
	TIM_TimeBaseInit(BASE_TIMER_INTR, &TIM_TimeBaseStructure); 			//???TIM3
 
	TIM_ITConfig(BASE_TIMER_INTR,TIM_IT_Update,ENABLE ); 					//??????

	NVIC_InitStructure.NVIC_IRQChannel = BASE_TIMER_INTR_IRQ;  			//TIM3??
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 	//?????
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  		//????
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//IRQ????
	NVIC_Init(&NVIC_InitStructure);  							//???NVIC

	TIM_Cmd(BASE_TIMER_INTR, ENABLE);  //??TIM3				 
}


void BASE_TIMER_INTR_IRQHandler(void)   //TIM3??
{
	CanRxMsg value;
	if (TIM_GetITStatus(BASE_TIMER_INTR,TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(BASE_TIMER_INTR, TIM_IT_Update);
//		while(out_queue(&value) == true)
//		{
//			deal_can_ext_frame(&value);
//		}
	}
}

#endif

