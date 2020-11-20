/*
 * it_implement.c
 *
 *  Created on: Oct 27, 2020
 *      Author: Dang Nam
 */

#include "ringbuffer.h"
#include "dma.h"
#include "usart.h"
#include "tx_dma_manage.h"
#include "step_motor.h"

/* External variables */
extern RingBuffer_t 		ringbuff_tx_pi;
extern RingBuffer_t 		ringbuff_tx_mega;
extern RingBuffer_t 		ringbuff_rx_pi;
extern RingBuffer_t 		ringbuff_rx_mega;
extern DMA_HandleTypeDef 	hdma_usart2_tx;
extern DMA_HandleTypeDef 	hdma_usart3_tx;
extern UART_HandleTypeDef 	huart2;
extern UART_HandleTypeDef 	huart3;
extern uint8_t pi_uart_dma_buff[DMA_TX_BUFF_SIZE];
extern uint8_t mega_uart_dma_buff[DMA_TX_BUFF_SIZE];

extern uint8_t receive_from_pi;
extern uint8_t receive_from_mega;

extern Step_t	stepDown; // Step 2 on board
extern Step_t	stepUp; // Step 1 on board
extern TIM_HandleTypeDef htim5; // Step 1 on board
extern TIM_HandleTypeDef htim9; // Step 2 on board

/* Implementation */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// From Rasberry Pi
	if(huart->Instance == huart2.Instance)
	{
		HAL_UART_Receive_IT(&huart2, &receive_from_pi, 1);
		ringBuff_PushChar(&ringbuff_rx_pi, receive_from_pi);
	}
	// From Arduino Mega
	if (huart->Instance == huart3.Instance)
	{
		HAL_UART_Receive_IT(&huart3, &receive_from_mega, 1);
		ringBuff_PushChar(&ringbuff_rx_mega, receive_from_mega);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	// To Rasberry Pi
	if(huart->Instance == huart2.Instance)
	{
		int32_t size;
		size = ringBuff_PopArray(&ringbuff_tx_pi,
								pi_uart_dma_buff,
								DMA_TX_BUFF_SIZE);
		if (size > 0)
			HAL_UART_Transmit_DMA(&huart2, pi_uart_dma_buff, size);
	}
	// To Arduino Mega
	if (huart->Instance == huart3.Instance)
	{
		int32_t size;
		size = ringBuff_PopArray(&ringbuff_tx_mega,
								mega_uart_dma_buff,
								DMA_TX_BUFF_SIZE);
		if (size > 0)
			HAL_UART_Transmit_DMA(&huart3, pi_uart_dma_buff, size);
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	// Step up
	if (htim->Instance == htim5.Instance) {
		stepUp.pulse_count_in_period++;
		if (stepUp.pulse_count_in_period >= stepUp.pulse_in_period) {
			stepUp.TIM_PULSE->CCR1 = MAX_TIMER_COUNT + 1;;
		}
	}
	// Step down
	if (htim->Instance == htim9.Instance) {
		stepDown.pulse_count_in_period++;
		if (stepDown.pulse_count_in_period >= stepDown.pulse_in_period) {
			stepDown.TIM_PULSE->CCR1 = MAX_TIMER_COUNT + 1;
		}
	}
}
