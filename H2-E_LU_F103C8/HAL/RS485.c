/*
 * RS-485.c
 *
 *  Created on: 2019年11月8日
 *      Author: 陆相如
 */
#include "RS485.h"

#include "bitband.h"

#define USARTx                   USART2
#define USARTx_GPIO              GPIOA
#define USARTx_CLK               RCC_APB1Periph_USART2
#define USARTx_GPIO_CLK          RCC_APB2Periph_GPIOA
#define USARTx_RxPin             GPIO_Pin_3
#define USARTx_TxPin             GPIO_Pin_2
#define USARTx_CtlPin            GPIO_Pin_0
#define USARTx_IRQn              USART2_IRQn
#define USARTx_IRQHandler        USART2_IRQHandler

#define        USARTX_RX_LEN      50           //??????
#define        USARTX_TX_LEN      50           //??????
#define     RS485_TX_EN        PAout(4)

u8 USARTX_RX_Buf[USARTX_RX_LEN];  //????
u8 USARTX_TX_Buf[USARTX_TX_LEN];  //????
u8 USARTX_RX_Data_Len = 0;        //??????????
u8 USARTX_TX_Data_Len = 0;        //?????????

void RS485_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitTypeStruct;
    USART_InitTypeDef USART_InitTypeStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_APB1PeriphClockCmd(USARTx_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(USARTx_GPIO_CLK, ENABLE);

    GPIO_InitTypeStruct.GPIO_Pin = USARTx_RxPin | USARTx_TxPin;
    GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(USARTx_GPIO, &GPIO_InitTypeStruct);

    USART_InitTypeStruct.USART_BaudRate = baudrate;
    USART_InitTypeStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitTypeStruct.USART_StopBits = USART_StopBits_1;
    USART_InitTypeStruct.USART_Parity = USART_Parity_No;
    USART_InitTypeStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitTypeStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USARTx, &USART_InitTypeStruct);
	
	  /*  USARTx */
    NVIC_InitStruct.NVIC_IRQChannel = USARTx_IRQn; //???? 2 ??
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3; //????? 3 ?
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2; //???? 2?
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE; //????????
    NVIC_Init(&NVIC_InitStruct);//??? NVIC ???
    
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE); //????
    USART_Cmd(USARTx, ENABLE); //????
    
    //USART_ClearFlag(USARTx, USART_FLAG_TC);//???????
    
    
    /**********************??485???*********************/

    GPIO_InitTypeStruct.GPIO_Pin = USARTx_CtlPin;    
    GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_Out_PP;   //???? ,PA4,485Ctr
    GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(USARTx_GPIO, &GPIO_InitTypeStruct);
    GPIO_ResetBits(USARTx_GPIO, USARTx_CtlPin);        //???????,????
		
}

void RS485_Send_Data(u8 *buf,u8 len)       //???????
{
    u8 t;
    for(t=0;t<len;t++)
    {
        while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
        USART_SendData(USARTx,buf[t]);
    }
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
}

void USARTx_IRQHandler(void)
{
    u8 res;
    if(USART_GetITStatus(USARTx, USART_IT_RXNE) != RESET) //?????
    {
        res =USART_ReceiveData(USARTx); //????????
        if(USARTX_RX_Data_Len<USARTX_RX_LEN)
        {
            USARTX_RX_Buf[USARTX_RX_Data_Len]=res; //???????
            USARTX_RX_Data_Len++; //?????? 1
					  RS485_Send_Data(USARTX_RX_Buf, USARTX_RX_Data_Len);
        }                 
    }
}

