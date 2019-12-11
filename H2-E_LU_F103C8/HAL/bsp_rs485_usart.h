#ifndef _BSP_RS485_USART_H_
#define _BSP_RS485_USART_H_
#include "stm32f10x.h"

typedef enum
{
    receive,
    transmit,
//    powerdown
} _bus_pin_status;





//void BSP_RS485_UARTx_Init(u32 br_num);
void BSP_RS485_Uart1_Init(void);
void BSP_RS485_Pin_Control(_bus_pin_status bus_pin_status);
void RS485_SendData(u8 *buf,u8 len);

void USART2_Initialise( u32 bound );
void USART2_SendData(u8 *buf,u8 len);
#endif
