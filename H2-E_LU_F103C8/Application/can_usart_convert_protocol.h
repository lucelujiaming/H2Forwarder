#ifndef _CAN_USART_CONVERT_PROTOCOL_H_
#define _CAN_USART_CONVERT_PROTOCOL_H_

#include "stm32f10x.h"
#include "bsp_can.h"
#include "xprintf.h"

#define CAN_HEAD                (uint8_t)0x55
#define CAN_TAIL                (uint8_t)0x55

#define CAN_TYPE_NORMAL_FRAME   (uint8_t)0xA0
#define CAN_TYPE_ERROR_FRAME    (uint8_t)0xA1
#define CAN_TYPE_CONFIG_FRAME   (uint8_t)0xA2

//  defined in stm32f10x_can.h
//	#define CAN_ErrorCode_NoErr           ((uint8_t)0x00) /*!< No Error */                // Always
//	#define	CAN_ErrorCode_StuffErr        ((uint8_t)0x10) /*!< Stuff Error */ 
//	#define	CAN_ErrorCode_FormErr         ((uint8_t)0x20) /*!< Form Error */ 
//	#define	CAN_ErrorCode_ACKErr          ((uint8_t)0x30) /*!< Acknowledgment Error */    // wire cut
//	#define	CAN_ErrorCode_BitRecessiveErr ((uint8_t)0x40) /*!< Bit Recessive Error */     // never happen
//	#define	CAN_ErrorCode_BitDominantErr  ((uint8_t)0x50) /*!< Bit Dominant Error */      // CAN Chip wreck
//	#define	CAN_ErrorCode_CRCErr          ((uint8_t)0x60) /*!< CRC Error  */ 
//	#define	CAN_ErrorCode_SoftwareSetErr  ((uint8_t)0x70) /*!< Software Set Error */ 
#define	CAN_ErrorCode_DataBufferErr       ((uint8_t)0x80) /*!< Data Buffer Error */ 

#define CAN_CLOSE           (uint8_t)0x00
#define CAN_SPEED_100K      (uint8_t)0x01
#define CAN_SPEED_125K      (uint8_t)0x02
#define CAN_SPEED_250K      (uint8_t)0x03
#define CAN_SPEED_500K      (uint8_t)0x04
#define CAN_SPEED_800K      (uint8_t)0x05
#define CAN_SPEED_1000K     (uint8_t)0x06

#pragma pack(1)
//正常帧数据类型：
typedef struct _normal_frame_t
{
    u8 can_head;    //0x55 
    u8 can_type;	//0xA0 : normal frame
    u32 can_id;
    u8 can_data[8];
    u8 can_cks;
    u8 can_tail;    //0x55
} _normal_frame;



//错误帧数据类型：
typedef struct _error_frame_t
{
    u8 can_head;    //0x55
    u8 can_type;    // 0xA1：error frame
    u8 can_errcode;    //
    u8 can_reserve;	
    u8 can_cks;
    u8 can_tail;    //0x55
} _error_frame;


//配置帧数据类型：
typedef struct _config_frame_t
{
    u8 can_head;    // 0 - 0x55
    u8 can_type;    // 1 - 0xA2:  config_frame
    u8 can_speed;   // 2 - 
    u8 can_reserve; // 3 - 
    u8 can_cks;     // 4 - 
    u8 can_tail;    // 5 - 0x55
} _config_frame;
#pragma pack() 

u8 deal_can_ext_frame(CanRxMsg* rxMsg);
u8 deal_can_error_frame(uint8_t error);
u8 set_can_baudrate_frame(u8 baudRate);

#endif
