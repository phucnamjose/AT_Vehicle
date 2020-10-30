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
		enum_DebugCmd cmd_code;
		mainTaskMail_t cmd_to_main;

		at_cmd 		= AtSerial_GetCommand(&atSerialPi);
		lenght 		= AtSerial_GetData(&atSerialPi, data_raw);
		cmd_code 	= AtSerial_HaldleCommand(at_cmd, data_raw, lenght, &cmd_to_main);

		if (cmd_code > CMD_NONE) {
			// 1. Forward to Arduino Mega
			if (cmd_code == FORWARD_MSG) {
				serial_sendArduinoMega(&(atSerialPi._serialRecvBytes[0]),
										atSerialPi._segmentLength);
			// 2. Send mail to main thread to make desision
			} else {
				mainTask_SendMail(&cmd_to_main);
			}
		}
	}
}

