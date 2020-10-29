/*
 * communicate_payload.h
 *
 *  Created on: Mar 2, 2020
 *      Author: Dang Nam
 */

#ifndef INC_COMMUNICATE_PAYLOAD_H_
#define INC_COMMUNICATE_PAYLOAD_H_

#include "def_myself.h"


#define START_CHAR		(0x24) // '$'
#define END_CHAR		(0x0A) // '\n'

#define MIN_MESSAGE_LENGHT	(6)

int32_t	packPayload		(uint8_t *input_buff, uint8_t *output_buff, int32_t len);
int32_t	unPackPayload	(uint8_t *message_buff, int32_t in_lenght);


#endif /* INC_COMMUNICATE_PAYLOAD_H_ */
