///*
// * rasberry_thread.c
// *
// *  Created on: Oct 27, 2020
// *      Author: Dang Nam
// */
//
#include "rasberry_thread.h"
#include "main_thread.h"
#include "ringbuffer.h"
#include "def_myself.h"
#include "string.h"
#include "cmsis_os.h"
#include "arduino.h"
#include "tx_dma_manage.h"
#include "usart.h"
#include "debug_cmd.h"

/* External Variables */
extern UART_HandleTypeDef huart2;
extern RingBuffer_t ringbuff_rx_pi;
extern AtSerial_t 	atSerialPi;
extern osMailQId mainTaskMailHandle;

uint8_t receive_from_pi;


/* Implementation */
void setupRasberryThread(void) {
	AtSerial_Init(&atSerialPi);
	HAL_UART_Receive_IT(&huart2, &receive_from_pi, 1);
}

void loopRasberryThread(void) {
	if (AtSerial_ReadCommand(&atSerialPi, &ringbuff_rx_pi)) {
		uint8_t at_cmd = 1;
		uint8_t data_raw[60];
		uint32_t lenght;
		enum_MessageClass_t msg_classify;
		mainTaskMail_t cmd_to_main;
		// Read
		at_cmd 			= AtSerial_GetCommand(&atSerialPi);
		lenght 			= AtSerial_GetData(&atSerialPi, data_raw);
		msg_classify 	= AtSerial_HaldleComputer(at_cmd,
												data_raw,
												lenght,
												&cmd_to_main);
		// Classify
		switch (msg_classify) {
			case MSG_WRONG:
			case MSG_ONPY_ME:
				break;
			case MSG_FORWARD:
				serial_sendArduinoMega(&(atSerialPi._serialRecvBytes[0]),
									atSerialPi._segmentLength);
				break;
			case MSG_MAIL_TO_MAINTASK:
				mainTask_SendMail(&cmd_to_main);
				break;
			default:
				break;
		}
	}
}

