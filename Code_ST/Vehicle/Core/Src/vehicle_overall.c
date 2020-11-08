/*
 * vehicle_overall.c
 *
 *  Created on: Oct 31, 2020
 *      Author: Dang Nam
 */

#include "vehicle_overall.h"
#include "pid_motor.h"
#include "fuzzy_controller.h"
#include "stanley_controller.h"



/* External Variables */
extern PID_t	pid_MR;
extern PID_t	pid_ML;
extern IMU      Mag;
extern GPS		GPS_NEO;


double	time_move_count;


void Vehicle_Drive_Auto(void) {

}


void Vehicle_Stop(void) {
	PID_Reset(&pid_MR);
	PID_Reset(&pid_ML);
}

void Vehicle_Forward(void) {
	PID_Setpoint(&pid_MR, 2);
	PID_Setpoint(&pid_ML, 2);
}

void Vehicle_Backward(void) {
	PID_Setpoint(&pid_MR, -1);
	PID_Setpoint(&pid_ML, -1);
}

void Vehicle_RotLeft(void) {
	PID_Setpoint(&pid_MR, 1);
	PID_Setpoint(&pid_ML, -1);
}

void Vehicle_RotRight(void) {
	PID_Setpoint(&pid_MR, -1);
	PID_Setpoint(&pid_ML, 1);
}

void Vehicle_ForwardLeft(void) {
	PID_Setpoint(&pid_MR, 2);
	PID_Setpoint(&pid_ML, 1);
}

void Vehicle_ForwardRight(void) {
	PID_Setpoint(&pid_MR, 1);
	PID_Setpoint(&pid_ML, 2);
}

void Vehicle_BackwardLeft(void) {
	PID_Setpoint(&pid_MR, -2);
	PID_Setpoint(&pid_ML, -1);
}

void Vehicle_BackwardRight(void) {
	PID_Setpoint(&pid_MR, -1);
	PID_Setpoint(&pid_ML, -2);
}







