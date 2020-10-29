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

}

void PID_Compute(PID_t *pid, DcServo_t *motor) {
	double P_part, I_part, D_part;
	// Previous
	pid->pre2_error = pid->pre_error;
	pid->pre_error 	= pid->error;
	pid->pre_output = pid->output;
	// Current
	pid->v_feedback = motor->current_v;
	pid->error 		= pid->v_setpoint - pid->v_feedback;
	// PID
	P_part = pid->Kp*(pid->error - pid->pre_error);
	I_part = 0.5*pid->Ki*BASIC_PERIOD*(pid->error + pid->pre_error);
	D_part = pid->Kd*(pid->error - 2*pid->pre_error + pid->pre2_error)/BASIC_PERIOD;
	pid->output = pid->pre_output + P_part + I_part + D_part;
	// Filter
	pid->output = filter(0.08, pid->output, pid->pre_output);
	// Update to motor
	motor->out_percent = pid->output;
}

void PID_Setpoint(PID_t *pid, double setpoint) {
	if (setpoint < 13.0) {
		pid->v_setpoint = setpoint;
	}
}

double	PID_GetSetpoint(PID_t *pid) {
	return pid->v_setpoint;
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
