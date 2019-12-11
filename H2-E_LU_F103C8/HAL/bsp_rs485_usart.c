#include "stm32f10x.h"
#include "bsp_rs485_usart.h"
#include "xprintf.h"
#include "can_usart_convert_protocol.h"

#if 0
u16 USART_RX_STA=0;       // 接收状态标记 
                          // 可以发现0~13位接受的是数据个数（填满相当于十进制的8191），
                          // ②那么既然0~13位数据量这么大，代码是如何实现对14、15位的修改呢？
						  // 废话不多说，请看实现代码
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符
#endif

u16 SET_CAN_BITRATE_HEAD_SEQ=0;  
u8  USART_RX_STA_LEN=0;
#define USART_REC_LEN  			200
u8  USART_RX_BUF[USART_REC_LEN]; 

static void BSP_RS485_Control_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd ( RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
void NVIC_Configuration_Uart1(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void BSP_RS485_Uart1_Init(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //开启USART1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	//配置PA9作为USART1　Tx
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA , &GPIO_InitStructure);
	//配置PA10作为USART1　Rx
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA , &GPIO_InitStructure);

	BSP_RS485_Control_Init();
	//配置USART1
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	//配置USART1时钟
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出

	USART_Init(USART1, &USART_InitStructure);
	USART_ClockInit(USART1, &USART_ClockInitStruct);

	NVIC_Configuration_Uart1();
	//使能USART1接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	//使能USART1
	USART_Cmd(USART1, ENABLE);

	BSP_RS485_Pin_Control(receive);
}


void BSP_RS485_Pin_Control(_bus_pin_status bus_pin_status)
{
    switch (bus_pin_status)
    {
    case receive:
//        GPIOA->BRR |= GPIO_BRR_BR_8;
		GPIO_ResetBits(GPIOA,GPIO_Pin_8);
        break;
    case transmit:
//        GPIOA->BSRR |= GPIO_BSRR_BS_8;
		GPIO_SetBits(GPIOA,GPIO_Pin_8);
        break;
    default:
        break;
    }
}

void RS485_SendData(u8 *buf,u8 len)       //发送完改为接收
{
    u8 t;
    BSP_RS485_Pin_Control(transmit); //设置为发送模式
    for(t=0;t<len;t++)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
        USART_SendData(USART1,buf[t]);
    }
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    // USARTx_RX_Data_Len=0;
    BSP_RS485_Pin_Control(receive);//设置为接收模式
}

void USART2_Initialise( u32 bound )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
    USART_InitTypeDef USART_InitStructure;

    /* Enable the USART2 Pins Software Remapping */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //开启USART2时钟

    /* Configure USART2 Rx (PA.03) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART2 Tx (PA.02) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

		//配置USART2
    USART_InitStructure.USART_BaudRate = bound;                //波特率9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;     //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;        //不使用奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//发送、接收使能
	//配置USART2时钟
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出

    USART_Init(USART2, &USART_InitStructure);
		USART_ClockInit(USART2, &USART_ClockInitStruct);

    /* Enable the USART2 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    /* Enable USART2 */
    USART_Cmd(USART2, ENABLE);
}

void USART2_IRQHandler(void)
{
	u8 res = 0;
	if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET)
	{
		// 接收中断(接收到的数据必须是 0x0d 0x0a 结尾)
		res = USART_ReceiveData(USART2); // 读取接收到的数据
		xprintf("USART2_IRQHandler data = %d", res);
		USART_RX_BUF[USART_RX_STA_LEN]=res ;
		
		if(USART_RX_STA_LEN >= sizeof(_config_frame) - 1)
		{
			res = USART_RX_BUF[2];
			set_can_baudrate_frame(res);
			USART_RX_STA_LEN = 0;
		}
		else 
		{
			USART_RX_STA_LEN++;
		}
	}
}


void USART2_SendData(u8 *buf,u8 len)       //发送完改为接收
{
    u8 t;
    for(t=0;t<len;t++)
    {
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
        USART_SendData(USART2,buf[t]);
    }
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);

    // USARTx_RX_Data_Len=0;
}
