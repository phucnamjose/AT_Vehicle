/*
 * FIFO_buff.c
 *
 *  Created on: Nov 23, 2020
 *      Author: Dang Nam
 */

#include "FIFO_buff.h"

FifoBuff_t fifo_delta_x = {0, {0}};
FifoBuff_t fifo_delta_y = {0, {0}};
FifoBuff_t fifo_delta_yaw = {0, {0}};

void 	FifoResetBuff(FifoBuff_t *fifo) {
	fifo->index_next = 0;
	for (uint8_t i = 0; i < SIZE_FIFO_BUFF; i++) {
		fifo->buff[i] = 0;
	}
}

void 	FifoPushSample(FifoBuff_t *fifo, double value) {
	fifo->buff[fifo->index_next] = value;
	fifo->index_next = (fifo->index_next + 1) % SIZE_FIFO_BUFF;
}

double 	FifoGetSum(FifoBuff_t *fifo) {
	double sum = 0;

	for (uint8_t i = 0; i < SIZE_FIFO_BUFF; i++) {
		sum += fifo->buff[i];
	}

	return sum;
}


