#include "string.h"
#include "stm32f10x.h"
#include "can_usart_convert_protocol.h"
#include "bsp_rs485_usart.h"
#include "xprintf.h"

u8 calc_checksum(u8 *buf,u8 len)
{
	u8 uCheckSum = buf[0];
	for(u8 i = 1; i <len; i++)
	{
	 	uCheckSum = uCheckSum + buf[i];
	}
	return uCheckSum; 
}

u8 deal_can_ext_frame(CanRxMsg* rxMsg)
{
	_normal_frame objNormalFrame ;
	u8 u8Len = sizeof(_normal_frame);
	
	u8 * pUSARTSendPtr = (u8 *)&objNormalFrame;
	if(rxMsg->IDE == CAN_ID_STD)
	{
		xprintf("deal_can_ext_frame STD");
	}
	else if(rxMsg->IDE == CAN_ID_EXT)
	{
		if(rxMsg->RTR == CAN_RTR_DATA)
		{
			objNormalFrame.can_head = CAN_HEAD; 
			objNormalFrame.can_type = CAN_TYPE_NORMAL_FRAME;
			objNormalFrame.can_id   = rxMsg->ExtId;
			objNormalFrame.can_data[0] = rxMsg->Data[0];
			objNormalFrame.can_data[1] = rxMsg->Data[1];
			objNormalFrame.can_data[2] = rxMsg->Data[2];
			objNormalFrame.can_data[3] = rxMsg->Data[3];
			objNormalFrame.can_data[4] = rxMsg->Data[4];
			objNormalFrame.can_data[5] = rxMsg->Data[5];
			objNormalFrame.can_data[6] = rxMsg->Data[6];
			objNormalFrame.can_data[7] = rxMsg->Data[7];
			objNormalFrame.can_cks = 0x00;  // Temp set
			objNormalFrame.can_tail = CAN_TAIL; 
			objNormalFrame.can_cks = calc_checksum(
					pUSARTSendPtr + 1, sizeof(_normal_frame) - 2);
			USART2_SendData(pUSARTSendPtr, u8Len);
			return 1;
		}
		else if(rxMsg->RTR == CAN_RTR_REMOTE)
		{
			xprintf("deal_can_ext_frame EXT_REMOTE");
		}
	}
	return 0;
}


u8 deal_can_error_frame(uint8_t error)
{
	_error_frame objErrorFrame ;
	u8 * pUSARTSendPtr = (u8 *)&objErrorFrame;

	objErrorFrame.can_head    = CAN_HEAD; 
	objErrorFrame.can_type    = CAN_TYPE_ERROR_FRAME;
	objErrorFrame.can_errcode = error;
	objErrorFrame.can_reserve = 0x00;
	objErrorFrame.can_cks     = 0x00;
	objErrorFrame.can_tail    = CAN_TAIL;
	objErrorFrame.can_cks = calc_checksum(
			pUSARTSendPtr + 1, sizeof(_error_frame) - 2);
	USART2_SendData(pUSARTSendPtr, sizeof(_error_frame));
	return 1;
}

u8 set_can_baudrate_frame(u8 baudRate)
{
	switch(baudRate){
	case CAN_CLOSE:
		BSP_CAN_DeInit();
		break;
	case CAN_SPEED_100K:
		BSP_CAN_DeInit();
		BSP_CAN_Config(baudRate);  // CAN_SJW_1tq, CAN_BS1_3tq, CAN_BS2_2tq, 60
		break;
	case CAN_SPEED_125K:
		BSP_CAN_DeInit();
		BSP_CAN_Config(baudRate);  // CAN_SJW_1tq, CAN_BS1_3tq, CAN_BS2_2tq, 48);
		break;
	case CAN_SPEED_250K:
		BSP_CAN_DeInit();
		BSP_CAN_Config(baudRate);  // CAN_SJW_1tq, CAN_BS1_3tq, CAN_BS2_2tq, 24);
		break;
	case CAN_SPEED_500K:
		BSP_CAN_DeInit();
		BSP_CAN_Config(baudRate);  // AN_SJW_1tq, CAN_BS1_6tq, CAN_BS2_1tq, 9);
		break;
	case CAN_SPEED_800K:
		BSP_CAN_DeInit();
		BSP_CAN_Config(baudRate);  // CAN_SJW_1tq, CAN_BS1_5tq, CAN_BS2_3tq, 5);
		break;
	case CAN_SPEED_1000K:
		BSP_CAN_DeInit();
		BSP_CAN_Config(baudRate);  // CAN_SJW_1tq, CAN_BS1_3tq, CAN_BS2_2tq, 6);
		break;
	default:
		xprintf("Not support baudRate");
		break;
	}
	return 1;
}

