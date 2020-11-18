/*
 * pid_heading.c
 *
 *  Created on: Nov 11, 2020
 *      Author: Dang Nam
 */

#include "pid_heading.h"
#include "pid_motor.h"
#include "system_params.h"
#include "stdlib.h"

/* External variables*/
extern PID_t		pid_MR;
extern PID_t		pid_ML;

/* Internal variables*/
HeadPID_t head_pid;

void	Head_PID_Init() {
	Head_PID_Reset(&head_pid);
	Head_PID_SetFactor(&head_pid,
						HEAD_PID_DEFAULT_KP,
						HEAD_PID_DEFAULT_KI,
						HEAD_PID_DEFAULT_KD);
}

void 	Head_PID_Reset(HeadPID_t *pid) {
	// Reset
	pid->error = 0;
	pid->pre_error = 0;
	pid->pre2_error = 0;
	pid->output = 0;
	pid->pre_output = 0;
	pid->v_setpoint = 0;

	pid->count = 0;
}

void Head_PID_SetFactor(HeadPID_t *pid, double kp, double ki, double kd) {
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
}

void	Head_PID_UpdateFeedback(HeadPID_t *pid, double feedback) {
	pid->v_feedback = feedback;
}

void	Head_PID_Compute(HeadPID_t *pid) {
	double P_part, I_part, D_part, output;
	// Previous
	pid->pre2_error = pid->pre_error;
	pid->pre_error 	= pid->error;
	pid->pre_output = pid->output;
	// Current
	pid->error 		= pid->v_setpoint - pid->v_feedback;
	// PID
	P_part = pid->Kp*(pid->error - pid->pre_error);
	I_part = 0.5*pid->Ki*HEAD_PERIOD*(pid->error + pid->pre_error);
	D_part = pid->Kd*(pid->error - 2*pid->pre_error + pid->pre2_error)/HEAD_PERIOD;
	output = pid->pre_output + P_part + I_part + D_part;

	// Saturation
	if (output > 0.6)
		output = 0.6;
	else if (output < -0.6)
		output = -0.6;

 	pid->output = output;
}

void	Head_PID_StartBlackBox(HeadPID_t *pid) {
	pid->run_black_box = TRUE;
	pid->count = 0;
}

uint8_t	Head_PID_RunBlackBox(HeadPID_t *pid) {
	if (pid->run_black_box == 0) {
		return FALSE;
	}
	pid->count++;
	if (pid->count > 0 && pid->count < 20) {
		Head_PID_SetPoint(pid, 0.5);

	} else if (pid->count < 30) {
		Head_PID_SetPoint(pid, 1);

	} else if (pid->count < 40) {
		Head_PID_SetPoint(pid, 0.8);

	} else if (pid->count < 54) {
		Head_PID_SetPoint(pid, 0.2);
	} else if (pid->count < 70) {
		Head_PID_SetPoint(pid, 0.9);

	} else if (pid->count < 96) {
		Head_PID_SetPoint(pid, 0.6);

	} else if (pid->count < 110) {
		Head_PID_SetPoint(pid, 0.1);

	} else if (pid->count < 140) {
		Head_PID_SetPoint(pid, 0.7);

	} else if (pid->count < 160) {
		Head_PID_SetPoint(pid, 0.05);

	} else if (pid->count < 180) {
		Head_PID_SetPoint(pid, 0.4);

	} else if (pid->count < 200) {
		Head_PID_SetPoint(pid, 1);

	} else {
		pid->count = 0;
		pid->run_black_box = FALSE;
	}
	return TRUE;
}

void	Head_PID_SetPoint(HeadPID_t *pid, double vel) {
	double vel_each = MPS2RPS(vel)*0.5;
	PID_Setpoint(&pid_MR, vel_each);
	PID_Setpoint(&pid_ML, -vel_each);
}


