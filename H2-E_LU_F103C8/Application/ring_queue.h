#ifndef _RING_QUEUE_H_
#define _RING_QUEUE_H_

#include "stm32f10x_can.h"

#define true 1
#define false 0
#define BUF_SIZE 512

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct _ring_queue
{
    CanRxMsg * data_buff_ptr;
    int front_index;
    int rear_index;
}ring_queue;


unsigned char init_queue(void);
// ���
unsigned char in_queue(CanRxMsg value);
// ���� 
unsigned char out_queue(CanRxMsg *value);
// ����
void print_queue(void);

extern ring_queue g_ring_queue ;

#ifdef __cplusplus
}
#endif

#endif
