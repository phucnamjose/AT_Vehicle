/*
 * rasberry_thread.c
 *
 *  Created on: Oct 27, 2020
 *      Author: Dang Nam
 */

#include "rasberry_thread.h"
#include "ringbuffer.h"
#include "def_myself.h"
#include "string.h"
#include "cmsis_os.h"
#include "arduino.h"
#include "tx_dma_manage.h"
#include "usart.h"

/* External Variables */
extern UART_HandleTypeDef huart2;
extern RingBuffer_t ringbuff_rx_pi;
extern AtSerial_t 	atSerialPi;

uint8_t receive_from_pi;


/* Implementation */
void setupRasberryThread(void) {
	AtSerial_Init(&atSerialPi);
	HAL_UART_Receive_IT(&huart2, &receive_from_pi, 1);
}

void loopRasberryThread(void) {
//	if (AtSerial_ReadCommand(&atSerialPi, &ringbuff_rx_pi)) {
//		//atSerialPi._tcpCommand
//	}
	int32_t distance = ringBuff_DistanceOf(&ringbuff_rx_pi, 'M');
	if ( distance > 0) {
		uint8_t temp[50];
		int32_t size = ringBuff_PopArray(&ringbuff_rx_pi, temp, distance+1);
		serial_sendRasberryPi(temp, size);
		__NOP();
	}
}

