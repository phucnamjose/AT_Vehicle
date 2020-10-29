/*
 * tx_dma_manage.h
 *
 *  Created on: Oct 27, 2020
 *      Author: Dang Nam
 */

#ifndef INC_TX_DMA_MANAGE_H_
#define INC_TX_DMA_MANAGE_H_


#include "def_myself.h"

#define DMA_TX_BUFF_SIZE (256)

/* Function Prototype */
uint8_t serial_sendRasberryPi(uint8_t *out_buff, int32_t lenght);
uint8_t serial_sendArduinoMega(uint8_t *out_buff, int32_t lenght);

#endif /* INC_TX_DMA_MANAGE_H_ */
