/*
 * main_thread.h
 *
 *  Created on: Oct 25, 2020
 *      Author: Dang Nam
 */

#ifndef INC_MAIN_THREAD_H_
#define INC_MAIN_THREAD_H_

#include "debug_cmd.h"
#include "def_myself.h"

/* Function Prototype */
void 	setupMainThread(void);
void 	loopMainThread(void);
void 	decisionAccordingCmd(mainTaskMail_t cmd);
uint8_t moveManualVehicle(enum_MoveManual move_type);
uint8_t moveAutoVehicle(double x, double y, uint8_t *target_data);
uint8_t	autoStart(void);
uint8_t	autoStop(void);
void 	handManualRobot(enum_HandManual hand_manual);
void 	handAutoRobot(enum_HandManual hand_manual);
void 	mainTask_SendMail(mainTaskMail_t *cmd_to_main);


#endif /* INC_MAIN_THREAD_H_ */
