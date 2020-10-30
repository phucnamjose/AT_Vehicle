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
void moveVehicle(enum_Move move_type);
void handRobot(enum_Hand hand_type);
void mainTask_SendMail(mainTaskMail_t *cmd_to_main);


#endif /* INC_MAIN_THREAD_H_ */
