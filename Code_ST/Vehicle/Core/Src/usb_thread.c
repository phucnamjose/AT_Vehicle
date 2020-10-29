/*
 * usb_thread.c
 *
 *  Created on: Oct 26, 2020
 *      Author: Dang Nam
 */

#include "usb_thread.h"
#include "ringbuffer.h"
#include "def_myself.h"
#include "string.h"
#include "communicate_payload.h"
#include "cmsis_os.h"
#include "debug_cmd.h"

/* External variable*/
extern RingBuffer_t ringbuff_rx_debug;
extern osMailQId mainTaskMailHandle;

void setupUsbThread(void) {

}

void loopUsbThread(void) {
	int32_t distance;
	// Check new message
	distance = ringBuff_DistanceOf(&ringbuff_rx_debug, END_CHAR);
	if ( -1 != distance ) {
		uint8_t temp[distance+1];
		int32_t ret;
		// Pop new message
		ringBuff_PopArray(&ringbuff_rx_debug, temp, distance + 1);
		ret = unPackPayload(temp, distance + 1);
		if( -1 == ret) {
			// Unpack fail
			__NOP();
		} else {
			// Unpack success
			enum_DebugCmd cmd_code;
			mainTaskMail_t cmd_to_main;

			cmd_code = MsgToCmd((char *)temp, &cmd_to_main);
			if (cmd_code > NONE) {
				mainTaskMail_t *mainMail;
				mainMail = NULL;

				// Allocate mail memory
				while (mainMail == NULL) {
					mainMail = osMailAlloc(mainTaskMailHandle, osWaitForever);
				}
				// Copy to mail
				memcpy(mainMail, &cmd_to_main, sizeof(mainTaskMail_t));
				// Send mail
				osStatus status_send;
				status_send = osMailPut(mainTaskMailHandle, mainMail);
			}
		}
	}
}
