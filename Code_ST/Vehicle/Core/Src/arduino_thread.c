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
		enum_DebugCmd cmd_code;
		mainTaskMail_t cmd_to_main;

		at_cmd 		= AtSerial_GetCommand(&atSerialMega);
		lenght 		= AtSerial_GetData(&atSerialMega, data_raw);
		cmd_code 	= AtSerial_HaldleCommand(at_cmd, data_raw, lenght, &cmd_to_main);

		if (cmd_code > CMD_NONE) {
			// 1. Forward to Arduino Mega
			if (cmd_code == FORWARD_MSG) {
				serial_sendRasberryPi(&(atSerialMega._serialRecvBytes[0]),
										atSerialMega._segmentLength);
			// 2. Send mail to main thread to make desision
			} else {
				mainTask_SendMail(&cmd_to_main);
			}
		}
	}
}



