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

extern PID_t		pid_MR;
extern PID_t		pid_ML;

double	sample_respond[500];
double	sample_output[500];
int32_t head_count;
uint8_t is_run_head;


void	Head_StartBlackBox(void) {
	is_run_head = TRUE;
	head_count = 0;
}

uint8_t	Head_RunBlackBox(void) {
	if (is_run_head == 0) {
		return FALSE;
	}
	head_count++;
	if (head_count > 0 && head_count < 20) {
		Head_SetVel(0.5);
		sample_output[head_count] = 0.5;
	} else if (head_count < 30) {
		Head_SetVel(1);
		sample_output[head_count] = 1;
	} else if (head_count < 40) {
		Head_SetVel(0.8);
		sample_output[head_count] = 0.8;
	} else if (head_count < 54) {
		Head_SetVel(0.2);
		sample_output[head_count] = 0.2;
	} else if (head_count < 70) {
		Head_SetVel(0.9);
		sample_output[head_count] = 0.9;
	} else if (head_count < 96) {
		Head_SetVel(0.6);
		sample_output[head_count] = 0.6;
	} else if (head_count < 110) {
		Head_SetVel(0.1);
		sample_output[head_count] = 0.1;
	} else if (head_count < 140) {
		Head_SetVel(0.7);
		sample_output[head_count] = 0.7;
	} else if (head_count < 160) {
		Head_SetVel(0.05);
		sample_output[head_count] = 0.05;
	} else if (head_count < 180) {
		Head_SetVel(0.4);
		sample_output[head_count] = 0.4;
	} else if (head_count < 200) {
		Head_SetVel(1);
		sample_output[head_count] = 1;
	} else {
		head_count = 0;
		is_run_head = FALSE;
	}
	return TRUE;
}

void	Head_SetVel(double vel) {
	double vel_each = abs(vel/2)*0.5;
	vel_each = MPS2RPS(vel_each);
	if (vel >= 0) {
		PID_Setpoint(&pid_MR, -vel_each);
		PID_Setpoint(&pid_ML, vel_each);
	} else {
		PID_Setpoint(&pid_MR, vel_each);
		PID_Setpoint(&pid_ML, -vel_each);
	}
}

void	Head_PushSample(double val) {
	sample_respond[head_count] = val;
}

double	Head_PopSample(int32_t index) {
	if (index < 0 || index > 499) {
		return -1;
	}
	return sample_respond[index];
}

double	Head_PopOutput(int32_t index) {
	if (index < 0 || index > 499) {
		return -1;
	}
	return sample_output[index];
}

uint8_t	Head_IsRun(void) {
	return is_run_head;
}
