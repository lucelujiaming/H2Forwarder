#include "ring_queue.h"
#include <stdio.h>

CanRxMsg g_objCanRxMsg[BUF_SIZE];
ring_queue g_ring_queue ;

unsigned char init_queue(void)
{
    g_ring_queue.data_buff_ptr = g_objCanRxMsg;
    if(g_ring_queue.data_buff_ptr != 0)     //队列内存分配成功
    {
        g_ring_queue.front_index = g_ring_queue.rear_index = 0; //初始化头尾指针 
		return true;
    }
    return false;
}

//判空
unsigned char isempty_queue(void)
{
    if(g_ring_queue.front_index == g_ring_queue.rear_index)
    {
        return true;
    }
    else
        return false;
}
 
//判满
unsigned char is_fullqueue(void)
{
	if((g_ring_queue.rear_index +1)%BUF_SIZE == g_ring_queue.front_index)
	{
		return true;
	}
	else
		return false;
}

//入队
 
unsigned char in_queue(CanRxMsg value)
{
	if(is_fullqueue() != true)        //队列未满
	{
		g_ring_queue.data_buff_ptr[g_ring_queue.rear_index] = value;
		g_ring_queue.rear_index = (g_ring_queue.rear_index + 1)%BUF_SIZE ;    //尾指针偏移 
		return true;
	}
	return false;
}
 

//出队 
unsigned char out_queue(CanRxMsg *value)
{
	if(isempty_queue() != true)        //队列未空
	{
		*value = g_ring_queue.data_buff_ptr[g_ring_queue.front_index];
		g_ring_queue.front_index = (g_ring_queue.front_index + 1)%BUF_SIZE ;
		return true;
	}
	return false;
}

void print_queue(void)
{
	printf(" -------------- ");
	if(isempty_queue() != true)
	{
		int ret=g_ring_queue.front_index;
		while(ret != g_ring_queue.rear_index)
		{ 
			printf("%d ",g_ring_queue.data_buff_ptr[ret].ExtId);
			ret=(ret+1)%BUF_SIZE;
		}
	}
	printf(" -------------- ");
}

