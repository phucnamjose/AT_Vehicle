/*
 * pid_motor.c
 *
 *  Created on: Oct 25, 2020
 *      Author: Dang Nam
 */


#include "pid_motor.h"

/* Global variables*/
PID_t	pid_MR;
PID_t	pid_ML;

/* Implementation */

void PID_Reset(PID_t *pid) {
	// Reset
	pid->error = 0;
	pid->pre_error = 0;
	pid->pre2_error = 0;
	pid->output = 0;
	pid->pre_output = 0;
	pid->v_setpoint = 0;

	pid->count = 0;
}

void PID_Compute(PID_t *pid, DcServo_t *motor) {
	double P_part, I_part, D_part, output;
	// Previous
	pid->pre2_error = pid->pre_error;
	pid->pre_error 	= pid->error;
	pid->pre_output = pid->output;
	// Current
	pid->v_feedback = motor->current_v*2*PI;
	pid->error 		= pid->v_setpoint - pid->v_feedback;
	// PID
	P_part = pid->Kp*(pid->error - pid->pre_error);
	I_part = 0.5*pid->Ki*BASIC_PERIOD*(pid->error + pid->pre_error);
	D_part = pid->Kd*(pid->error - 2*pid->pre_error + pid->pre2_error)/BASIC_PERIOD;
	output = pid->pre_output + P_part + I_part + D_part;

	// Saturation
	if (output > 0.6)
		output = 0.6;
	else if (output < 0)
		output = 0;

	// Filter
	output = filter(0.08, output, pid->pre_output);

 	pid->output = output;
	// Update to motor
	motor->out_percent = pid->output;
}

void PID_Setpoint(PID_t *pid, double setpoint) {
	if (setpoint < 2) {
		pid->v_setpoint = setpoint*2*PI;
	}
}

double	PID_GetSetpoint(PID_t *pid) {
	return pid->v_setpoint/(2*PI);
}

void PID_SetFactor(PID_t *pid, double kp, double ki, double kd) {
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
}

void	PID_GetFactor(PID_t *pid, double *kp, double *ki, double *kd) {
	*kp = pid->Kp;
	*ki = pid->Ki;
	*kd = pid->Kd;
}



void	PID_TestSquareWave(PID_t *pid, DcServo_t *motor) {
	pid->count++;
	if (pid->count == 100)
	{
		pid->output = 0.5;
		// Update to motor
		motor->out_percent = pid->output;
	} else if (pid->count == 200) {
		pid->output = 0;
		// Update to motor
		motor->out_percent = pid->output;
		pid->count = 0;
	}
}

double	PID_GetOutput(PID_t *pid) {
	return pid->output;
}
