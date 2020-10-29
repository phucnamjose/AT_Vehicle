/*
 * main_thread.h
 *
 *  Created on: Oct 25, 2020
 *      Author: Dang Nam
 */

#ifndef INC_MAIN_THREAD_H_
#define INC_MAIN_THREAD_H_

#include "debug_cmd.h"

/* Function Prototype */
void setupMainThread(void);
void loopMainThread(void);
void decisionAccordingCmd(mainTaskMail_t cmd);

#endif /* INC_MAIN_THREAD_H_ */
