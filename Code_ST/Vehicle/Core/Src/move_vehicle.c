/*
 * move_vehicle.c
 *
 *  Created on: Oct 29, 2020
 *      Author: Dang Nam
 */

#include "move_vehicle.h"
#include "pid_motor.h"

extern PID_t	pid_MR;
extern PID_t	pid_ML;


void Vehicle_Stop(void) {
	PID_Reset(&pid_MR);
	PID_Reset(&pid_ML);
}

void Vehicle_Forward(void) {
	PID_Setpoint(&pid_MR, 3);
	PID_Setpoint(&pid_ML, 3);
}

void Vehicle_Backward(void) {
	PID_Setpoint(&pid_MR, -2);
	PID_Setpoint(&pid_ML, -2);
}

void Vehicle_RotLeft(void) {
	PID_Setpoint(&pid_MR, 2);
	PID_Setpoint(&pid_ML, -2);
}

void Vehicle_RotRight(void) {
	PID_Setpoint(&pid_MR, -2);
	PID_Setpoint(&pid_ML, 2);
}

void Vehicle_ForwardLeft(void) {
	PID_Setpoint(&pid_MR, 3);
	PID_Setpoint(&pid_ML, 2);
}

void Vehicle_ForwardRight(void) {
	PID_Setpoint(&pid_MR, 2);
	PID_Setpoint(&pid_ML, 3);
}

void Vehicle_BackwardLeft(void) {
	PID_Setpoint(&pid_MR, -3);
	PID_Setpoint(&pid_ML, -2);
}

void Vehicle_BackwardRight(void) {
	PID_Setpoint(&pid_MR, -2);
	PID_Setpoint(&pid_ML, -3);
}



