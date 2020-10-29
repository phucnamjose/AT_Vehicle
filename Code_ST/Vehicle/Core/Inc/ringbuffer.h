/*
 * ringbuffer.h
 *
 *  Created on: Feb 28, 2020
 *      Author: Dang Nam
 */

#ifndef INC_RINGBUFFER_H_
#define INC_RINGBUFFER_H_

#include "def_myself.h"


#define RINGBUFFER_SIZE		(1024)



typedef struct
{
	uint8_t 	Array[RINGBUFFER_SIZE];
	int32_t 	head;
	int32_t 	tail;
	uint8_t 	isFull_Flag;
	uint8_t 	isEmpty_Flag;
}RingBuffer_t;


/* USER CODE BEGIN Prototypes */
uint8_t		ringBuff_PushChar		(RingBuffer_t *ringbuff, uint8_t data);
uint8_t 	ringBuff_PopChar		(RingBuffer_t *ringbuff, uint8_t *ptr_data);
int32_t		ringBuff_PushArray		(RingBuffer_t *ringbuff, uint8_t *ptr_data, int32_t len);
int32_t		ringBuff_PopArray		(RingBuffer_t *ringbuff, uint8_t *ptr_data, int32_t len);
uint8_t		ringBuff_IsFull			(RingBuffer_t ringbuff);
uint8_t		ringBuff_IsEmpty		(RingBuffer_t ringbuff);
int32_t		ringBuff_DistanceOf		(RingBuffer_t *ringbuff, uint8_t cmp_char);
int32_t		ringBuff_HowManySpace	(RingBuffer_t ringbuff);

#endif /* INC_RINGBUFFER_H_ */
