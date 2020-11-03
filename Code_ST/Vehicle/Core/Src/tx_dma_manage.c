/*
 * tx_dma_manage.c
 *
 *  Created on: Oct 27, 2020
 *      Author: Dang Nam
 */

#include "tx_dma_manage.h"
#include "def_myself.h"
#include "ringbuffer.h"
#include "stdio.h"
#include "string.h"
#include "dma.h"
#include "usart.h"
#include "cmsis_os.h"


/* External variables */
extern RingBuffer_t 		ringbuff_tx_pi;
extern RingBuffer_t 		ringbuff_tx_mega;
extern UART_HandleTypeDef 	huart2;
extern UART_HandleTypeDef 	huart3;
extern DMA_HandleTypeDef 	hdma_usart2_tx;
extern DMA_HandleTypeDef 	hdma_usart3_tx;
// Mutex
extern osMutexId txPiMutexHandle;
extern osMutexId txMegaMutexHandle;

/* Internal variables */
uint8_t pi_uart_dma_buff[DMA_TX_BUFF_SIZE];
uint8_t mega_uart_dma_buff[DMA_TX_BUFF_SIZE];

/* Implementation */
uint8_t serial_sendRasberryPi(uint8_t *out_buff, int32_t lenght) {
	int32_t res;
	uint8_t	ret_val;
	// lock mutex
	osMutexWait(txPiMutexHandle, osWaitForever);
	// Write to tx buffer and enable DMA
	res = ringBuff_PushArray(&ringbuff_tx_pi, out_buff, lenght);
	if (res != lenght)
		ret_val = FALSE;
	else {
		if (HAL_DMA_GetState(&hdma_usart2_tx) == HAL_DMA_STATE_READY) {
			int32_t size;
			size = ringBuff_PopArray(&ringbuff_tx_pi,
									pi_uart_dma_buff,
									DMA_TX_BUFF_SIZE);
			HAL_UART_Transmit_DMA(&huart2, pi_uart_dma_buff, size);
		}
		ret_val = TRUE;
	}
	// unlock mutex
	osMutexRelease(txPiMutexHandle);

	return ret_val;
}

uint8_t serial_sendArduinoMega(uint8_t *out_buff, int32_t lenght) {
	int32_t res;
	uint8_t	ret_val;
	// lock mutex
	osMutexWait(txMegaMutexHandle, osWaitForever);
	// Write to tx buffer and enable DMA
	res = ringBuff_PushArray(&ringbuff_tx_mega, out_buff, lenght);
	if (res != lenght)
		ret_val = FALSE;
	else {
		if (HAL_DMA_GetState(&hdma_usart3_tx) == HAL_DMA_STATE_READY) {
			int32_t size;
			size = ringBuff_PopArray(&ringbuff_tx_mega,
									mega_uart_dma_buff,
									DMA_TX_BUFF_SIZE);
			HAL_UART_Transmit_DMA(&huart3, mega_uart_dma_buff, size);
		}
		ret_val = TRUE;
	}
	// unlock mutex
	osMutexRelease(txMegaMutexHandle);

	return ret_val;
}


