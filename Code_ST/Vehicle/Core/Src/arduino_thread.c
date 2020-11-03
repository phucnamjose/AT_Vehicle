///*
// * arduino_thread.c
// *
// *  Created on: Oct 28, 2020
// *      Author: Dang Nam
// */
//
#include "arduino_thread.h"
#include "main_thread.h"
#include "ringbuffer.h"
#include "def_myself.h"
#include "string.h"
#include "cmsis_os.h"
#include "arduino.h"
#include "tx_dma_manage.h"
#include "usart.h"
//
//
///* External Variables */
extern UART_HandleTypeDef huart3;
extern RingBuffer_t ringbuff_rx_mega;
extern AtSerial_t atSerialMega;
//
uint8_t receive_from_mega;

/* Implementation */
void setupArduinoThread(void) {
	AtSerial_Init(&atSerialMega);
	HAL_UART_Receive_IT(&huart3, &receive_from_mega, 1);
}

void loopArduinoThread(void) {
	if (AtSerial_ReadCommand(&atSerialMega, &ringbuff_rx_mega)) {
		uint8_t at_cmd;
		uint8_t data_raw[60];
		uint32_t lenght;
		enum_MessageClass_t msg_classify;
		mainTaskMail_t cmd_to_main;
		// Read
		at_cmd 			= AtSerial_GetCommand(&atSerialMega);
		lenght 			= AtSerial_GetData(&atSerialMega, data_raw);
		msg_classify 	= AtSerial_HaldleArduino(at_cmd,
												data_raw,
												lenght,
												&cmd_to_main);
		// Classify
		switch (msg_classify) {
			case MSG_WRONG:
			case MSG_ONPY_ME:
				break;
			case MSG_FORWARD:
				serial_sendRasberryPi(&(atSerialMega._serialRecvBytes[0]),
									atSerialMega._segmentLength);
				break;
			case MSG_MAIL_TO_MAINTASK:
				mainTask_SendMail(&cmd_to_main);
				break;
			default:
				break;
		}
	}
}



