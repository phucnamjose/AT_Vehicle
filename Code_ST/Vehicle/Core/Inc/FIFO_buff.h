/*
 * FIFO_buff.h
 *
 *  Created on: Nov 23, 2020
 *      Author: Dang Nam
 */

#ifndef INC_FIFO_BUFF_H_
#define INC_FIFO_BUFF_H_

#include "system_params.h"
#include "def_myself.h"

#define SIZE_FIFO_BUFF	DELAY_SAMPLE_LIDAR

/* Object Form */
typedef struct FifoBuff_t
{
	uint8_t		index_next;
	double		buff[SIZE_FIFO_BUFF];
}FifoBuff_t;

/* Function Prototypes */
void 	FifoResetBuff(FifoBuff_t *fifo);
void 	FifoPushSample(FifoBuff_t *fifo, double value);
double 	FifoGetSum(FifoBuff_t *fifo);


#endif /* INC_FIFO_BUFF_H_ */
