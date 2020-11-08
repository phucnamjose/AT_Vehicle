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
	pid->v_feedback = motor->current_v;
	pid->error 		= pid->v_setpoint - pid->v_feedback;
	// PID
	P_part = pid->Kp*(pid->error - pid->pre_error);
	I_part = 0.5*pid->Ki*BASIC_PERIOD*(pid->error + pid->pre_error);
	D_part = pid->Kd*(pid->error - 2*pid->pre_error + pid->pre2_error)/BASIC_PERIOD;
	output = pid->pre_output + P_part + I_part + D_part;

	// Saturation
	if (output > 0.6)
		output = 0.6;
	else if (output < -0.6)
		output = -0.6;

	// Filter
	//output = filter(0.08, output, pid->pre_output);

 	pid->output = output;
	// Update to motor
	motor->out_percent = pid->output;
}

void PID_Setpoint(PID_t *pid, double setpoint) {
	if (setpoint >= -1.5 && setpoint <= 1.5) {
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

int32_t	PID_GetCount(PID_t *pid) {
	return pid->count;
}


void	PID_StartBlackBox(PID_t *pid) {
	pid->run_black_box = 1;
	pid->count = 0;
}

uint8_t	PID_RunBlackBox(PID_t *pid, DcServo_t *motor) {
	if (pid->run_black_box == 0) {
		return FALSE;
	}
	pid->count++;
	if (pid->count > 0 && pid->count < 90) {
		pid->output = 0.3;
		motor->out_percent = pid->output;
	} else if (pid->count < 150) {
		pid->output = 0.7;
		motor->out_percent = pid->output;
	} else if (pid->count < 200) {
		pid->output = 0.6;
		motor->out_percent = pid->output;
	} else if (pid->count < 270) {
		pid->output = 0.7;
		motor->out_percent = pid->output;
	} else if (pid->count < 350) {
		pid->output = 0.1;
		motor->out_percent = pid->output;
	} else if (pid->count < 480) {
		pid->output = 0.5;
		motor->out_percent = pid->output;
	} else if (pid->count < 500) {
		pid->output = 0.35;
		motor->out_percent = pid->output;
	} else if (pid->count < 700) {
		pid->output = 0.7;
		motor->out_percent = pid->output;
	} else if (pid->count < 800) {
		pid->output = 0.15;
		motor->out_percent = pid->output;
	} else if (pid->count < 900) {
		pid->output = 0.25;
		motor->out_percent = pid->output;
	} else if (pid->count < 1000) {
		pid->output = 0.6;
		motor->out_percent = pid->output;
	} else {
		pid->count = 0;
		pid->output = 0;
		motor->out_percent = pid->output;
		pid->run_black_box = 0;
	}
	return TRUE;
}


