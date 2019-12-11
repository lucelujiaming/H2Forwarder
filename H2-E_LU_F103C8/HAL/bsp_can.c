// #include "board_config.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_can.h"
#include "stm32f10x_rcc.h"
#include "bsp_can.h"
#include "xprintf.h"
#include "misc.h"
#include "can_usart_convert_protocol.h"
#include "ring_queue.h"

#define BSP_CAN
/*******************************************************/
//CAN related definition
//#define CANx                       			CAN1
#define CAN_CLK                    			RCC_APB1Periph_CAN1
// CAN Receive Interrupt
#define CAN_RX_IRQ							USB_LP_CAN1_RX0_IRQn
// CAN Receive Interrupt Function
#define CAN_RX_IRQHandler					USB_LP_CAN1_RX0_IRQHandler  // CAN1_RX1_IRQHandler // 

//CAN IO:
#define CAN_RX_PIN                 GPIO_Pin_11
#define CAN_TX_PIN                 GPIO_Pin_12
#define CAN_TX_GPIO_PORT          GPIOA
#define CAN_RX_GPIO_PORT          GPIOA
#define CAN_TX_GPIO_CLK           RCC_APB2Periph_GPIOA
#define CAN_RX_GPIO_CLK           RCC_APB2Periph_GPIOA
#define CAN_RX_SOURCE              GPIO_PinSource11
#define CAN_TX_SOURCE              GPIO_PinSource12

#define CAN_STB_PIN                GPIO_Pin_9
#define CAN_STB_GPIO_PORT          GPIOA
#define CAN_STB_GPIO_CLK           RCC_APB2Periph_GPIOA

#define CAN_NORMAL			GPIO_ResetBits(CAN_STB_GPIO_PORT,CAN_STB_PIN)
#define CAN_STANDBY			GPIO_SetBits(CAN_STB_GPIO_PORT,CAN_STB_PIN)
/*******************************************************/

#ifdef BSP_CAN

/**
  ******************************************************************************
  * @file    bsp_debug_usart.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   can驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火  STM32 F429 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "bsp_can.h"

#if defined(FreeRTOS)
#include "FreeRTOS.h"
#include "queue.h"
#endif


/*
 * 函数名：CAN_GPIO_Config
 * 描述  ：CAN的GPIO 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_GPIO_Config(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;   	

	// RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  /* 使能GPIO时钟*/
  RCC_APB2PeriphClockCmd(CAN_TX_GPIO_CLK|CAN_RX_GPIO_CLK|CAN_STB_GPIO_CLK, ENABLE);

  //CAN STB
	GPIO_InitStructure.GPIO_Pin = CAN_STB_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CAN_STB_GPIO_PORT, &GPIO_InitStructure);
	CAN_NORMAL;


	/* 配置 CAN TX 引脚 */
	GPIO_InitStructure.GPIO_Pin = CAN_TX_PIN;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(CAN_TX_GPIO_PORT, &GPIO_InitStructure);
	// GPIO_ResetBits(GPIOA,CAN_TX_PIN);

	/* 配置 CAN RX 引脚 */
	GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;

  GPIO_Init(CAN_RX_GPIO_PORT, &GPIO_InitStructure);
}

/*
 * 函数名：CAN_NVIC_Config
 * 描述  ：CAN的NVIC 配置,第1优先级组，0，0优先级
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Configure one bit for preemption priority */
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/*中断设置*/
	NVIC_InitStructure.NVIC_IRQChannel = CAN_RX_IRQ;	   //CAN RX中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			   
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/*CAN通信中断使能*/
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	
	/*  */
	CAN_ITConfig(CAN1, CAN_IT_BOF | CAN_IT_ERR | CAN_IT_EPV | CAN_IT_EWG | CAN_IER_FFIE0
									  // | CAN_IT_LEC | CAN_IT_WKU | CAN_IT_SLK
										, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_SCE_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*
 * 函数名：CAN_Mode_Config
 * 描述  ：CAN的模式 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_Mode_Config(uint8_t bitRate)
{
	CAN_InitTypeDef 	   CAN_InitStructure;
	/************************CAN通信参数设置**********************************/
	/* Enable CAN clock */
	RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);

	/*CAN寄存器初始化*/
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);

	/*CAN单元初始化*/
	CAN_InitStructure.CAN_TTCM=DISABLE; 		   //MCR-TTCM  关闭时间触发通信模式使能
	CAN_InitStructure.CAN_ABOM=DISABLE; 		   //MCR-ABOM  使能自动离线管理 
	CAN_InitStructure.CAN_AWUM=DISABLE;			   //MCR-AWUM  使用自动唤醒模式
	CAN_InitStructure.CAN_NART=DISABLE; 		   //MCR-NART  禁止报文自动重传   
	CAN_InitStructure.CAN_RFLM=DISABLE; 		   //MCR-RFLM  接收FIFO 不锁定 溢出时新报文会覆盖原有报文  
	CAN_InitStructure.CAN_TXFP=ENABLE; 		   //MCR-TXFP  发送FIFO优先级 取决于报文标示符 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //正常工作模式

	
	if(bitRate == CAN_SPEED_100K)
	{
		//CAN Baudrate=FCLK1/(1+BS1+BS2)/CAN_PRESCALER=36/6/60=100K
		CAN_InitStructure.CAN_SJW= CAN_SJW_1tq;    //BTR-SJW 重新同步跳跃宽度 1个时间单元
		CAN_InitStructure.CAN_BS1= CAN_BS1_3tq;    //BTR-TS1 时间段1 占用了3个时间单元
		CAN_InitStructure.CAN_BS2= CAN_BS2_2tq;    //BTR-TS1 时间段2 占用了2个时间单元	

		/* (1MBps为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 =36 MHz) */	 
		CAN_InitStructure.CAN_Prescaler = 60; 
	}
	else if(bitRate == CAN_SPEED_125K)
	{
		//CAN Baudrate=FCLK1/(1+BS1+BS2)/CAN_PRESCALER=36/6/48=125K
		CAN_InitStructure.CAN_SJW= CAN_SJW_1tq;    //BTR-SJW 重新同步跳跃宽度 1个时间单元
		CAN_InitStructure.CAN_BS1= CAN_BS1_3tq;    //BTR-TS1 时间段1 占用了3个时间单元
		CAN_InitStructure.CAN_BS2= CAN_BS2_2tq;    //BTR-TS1 时间段2 占用了2个时间单元	

		/* (1MBps为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 =36 MHz) */	 
		CAN_InitStructure.CAN_Prescaler = 48; 
	}
	else if(bitRate == CAN_SPEED_250K)
	{
		//CAN Baudrate=FCLK1/(1+BS1+BS2)/CAN_PRESCALER=36/6/24=250K
		CAN_InitStructure.CAN_SJW= CAN_SJW_1tq;    //BTR-SJW 重新同步跳跃宽度 1个时间单元
		CAN_InitStructure.CAN_BS1= CAN_BS1_3tq;    //BTR-TS1 时间段1 占用了3个时间单元
		CAN_InitStructure.CAN_BS2= CAN_BS2_2tq;    //BTR-TS1 时间段2 占用了2个时间单元	

		/* (1MBps为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 =36 MHz) */	 
		CAN_InitStructure.CAN_Prescaler = 24; 
	}
	else if(bitRate == CAN_SPEED_500K)
	{
		//CAN Baudrate=FCLK1/(1+BS1+BS2)/CAN_PRESCALER=36/8/9=500K
		CAN_InitStructure.CAN_SJW= CAN_SJW_1tq;    //BTR-SJW 重新同步跳跃宽度 1个时间单元
		CAN_InitStructure.CAN_BS1= CAN_BS1_6tq;    //BTR-TS1 时间段1 占用了6个时间单元
		CAN_InitStructure.CAN_BS2= CAN_BS2_1tq;    //BTR-TS1 时间段2 占用了1个时间单元	

		/* (1MBps为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 =36 MHz) */	 
		CAN_InitStructure.CAN_Prescaler = 9; 
	}
	else if(bitRate == CAN_SPEED_800K)
	{
		//CAN Baudrate=FCLK1/(1+BS1+BS2)/CAN_PRESCALER=36/9/5=800K
		CAN_InitStructure.CAN_SJW= CAN_SJW_1tq;    //BTR-SJW 重新同步跳跃宽度 1个时间单元
		CAN_InitStructure.CAN_BS1= CAN_BS1_5tq;    //BTR-TS1 时间段1 占用了5个时间单元
		CAN_InitStructure.CAN_BS2= CAN_BS2_3tq;    //BTR-TS1 时间段2 占用了3个时间单元	

		/* (1MBps为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 =36 MHz) */	 
		CAN_InitStructure.CAN_Prescaler = 5; 
	}
	else if(bitRate == CAN_SPEED_1000K)
	{
		//CAN Baudrate=FCLK1/(1+BS1+BS2)/CAN_PRESCALER=36/6/6=1000K
		CAN_InitStructure.CAN_SJW= CAN_SJW_1tq;    //BTR-SJW 重新同步跳跃宽度 1个时间单元
		CAN_InitStructure.CAN_BS1= CAN_BS1_3tq;    //BTR-TS1 时间段1 占用了3个时间单元
		CAN_InitStructure.CAN_BS2= CAN_BS2_2tq;    //BTR-TS1 时间段2 占用了2个时间单元	

		/* (1MBps为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 =36 MHz) */	 
		CAN_InitStructure.CAN_Prescaler = 6; 
	}
	CAN_Init(CAN1, &CAN_InitStructure);
}



/*
 * 函数名：CAN_Filter_Config
 * 描述  ：CAN的筛选器 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */

static void CAN_Filter_Config(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/*CAN筛选器初始化*/
	CAN_FilterInitStructure.CAN_FilterNumber=0;						//筛选器组0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//工作在掩码模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//筛选器位宽为单个32位。
	/* 使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */
	//PF<240 目标地址=本设备可以通过
	CAN_FilterInitStructure.CAN_FilterIdHigh= 0x0000; // ((CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF0000)>>16;		//要筛选的ID高位 
	CAN_FilterInitStructure.CAN_FilterIdLow= 0x0000; // (CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //要筛选的ID低位 
//	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0x0000;			//筛选器高16位每位必须匹配
//	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;			//筛选器低16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow =0x0000;	//CAN_ID_EXT|CAN_RTR_DATA
    
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//筛选器被关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//使能筛选器
	CAN_FilterInit(&CAN_FilterInitStructure);
}


/*
 * 函数名：BSP_CAN_Config
 * 描述  ：完整配置CAN的功能
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void BSP_CAN_Config(uint8_t bitRate)
{
  CAN_GPIO_Config();
  CAN_Mode_Config(bitRate);
  CAN_NVIC_Config();
  CAN_Filter_Config();  
}


void BSP_CAN_DeInit(void)
{
	  GPIO_DeInit(CAN_RX_GPIO_PORT);
    CAN_DeInit(CAN1);
}


/******************************************************************************
*CAN 接收中断函数
*******************************************************************************/

void USB_HP_CAN1_TX_IRQHandler(void)
{
		xprintf("get USB_HP_CAN1_TX_IRQHandler");
}

void CAN_RX_IRQHandler(void)
{
	CanRxMsg rxMessage;  
	CAN_Receive(CAN1, CAN_FIFO0, &rxMessage);
		xprintf("get CAN1_RX1_IRQHandler");
//	if(rxMessage.IDE == CAN_ID_STD)
//	{
//		xprintf("get STD");
//	}
//	else if(rxMessage.IDE == CAN_ID_EXT)
//	{
//		if(rxMessage.RTR == CAN_RTR_DATA)
//		{
//			deal_can_ext_frame(&rxMessage);
//		}
//		else if(rxMessage.RTR == CAN_RTR_REMOTE)
//		{
//			xprintf("get EXT_REMOTE");
//		}
//	}

//	deal_can_ext_frame(&rxMessage);
	if(in_queue(rxMessage) == false)
	{
		deal_can_error_frame(CAN_ErrorCode_DataBufferErr);
		init_queue();
	}
}
	

void CAN1_SCE_IRQHandler(void)
{
	static uint8_t error = 0;
	if(CAN_GetFlagStatus(CAN1, CAN_FLAG_BOF) == SET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_BOF);
	}
	else if(CAN_GetITStatus(CAN1, CAN_IT_ERR) == SET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_ERR);
	}        
	else if(CAN_GetFlagStatus(CAN1, CAN_FLAG_EPV) == SET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_EPV);
	}
	else if(CAN_GetFlagStatus(CAN1, CAN_FLAG_EWG) == SET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_EWG);
	}
	else
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_EWG);
		CAN_ClearITPendingBit(CAN1, CAN_IT_EPV);
		CAN_ClearITPendingBit(CAN1, CAN_IT_BOF);
		CAN_ClearITPendingBit(CAN1, CAN_IT_ERR);
	}
	error = 0;
	error = CAN_GetLastErrorCode(CAN1);        
	error = error;        
	// BSP_CAN_Config();
	
  // deal error_frame
	xprintf("get CAN1_SCE_IRQHandler");
	deal_can_error_frame(error);
}

/******************************************************************************
*CAN 发送中断函数
*******************************************************************************/
	
//void CAN1_TX_IRQHandler(void)
//{
//	J1939_ISR(); 
//}


u8 BSP_CAN_SetMsg(u8 Data1,u8 Data2)
{ 
    u8 mbox;
    u16 i=0; 
    CanTxMsg TxMessage;  

    TxMessage.StdId=0x0000;     //标准标识符为0x00
    TxMessage.ExtId=0x1314;     //扩展标识符0x1311,可以更改该标识符以示区分不同从机
    TxMessage.IDE=CAN_ID_EXT;   //使用扩展标识符
    TxMessage.RTR=CAN_RTR_DATA; //为数据帧
    TxMessage.DLC=2;            //消息的数据长度为2个字节
    TxMessage.Data[0]=Data1;    //第一个字节数据
    TxMessage.Data[1]=Data2;    //第二个字节数据 

    //发送数据
    mbox= CAN_Transmit(CAN1, &TxMessage);  
    while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))
        i++;    //等待发送结束
    if(i>=0XFFF)
        return 0;
    return 1;
}

/**************************END OF FILE************************************/

#endif
