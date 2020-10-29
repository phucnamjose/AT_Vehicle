/*
 * arduino_thread.c
 *
 *  Created on: Oct 28, 2020
 *      Author: Dang Nam
 */

#include "arduino_thread.h"
#include "ringbuffer.h"
#include "def_myself.h"
#include "string.h"
#include "cmsis_os.h"
#include "arduino.h"
#include "tx_dma_manage.h"
#include "usart.h"


/* External Variables */
extern UART_HandleTypeDef huart3;
extern RingBuffer_t ringbuff_rx_mega;
extern AtSerial_t atSerialMega;

uint8_t receive_from_mega;

/* Implementation */
void setupArduinoThread(void) {
	AtSerial_Init(&atSerialMega);
	HAL_UART_Receive_IT(&huart3, &receive_from_mega, 1);
}

void loopArduinoThread(void) {
	if (AtSerial_ReadCommand(&atSerialMega, &ringbuff_rx_mega)) {
		//atSerialPi._tcpCommand
	}
}



