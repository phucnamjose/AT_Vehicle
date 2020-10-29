/*
 * pid_motor.h
 *
 *  Created on: Oct 25, 2020
 *      Author: Dang Nam
 */

#ifndef INC_PID_MOTOR_H_
#define INC_PID_MOTOR_H_

#include "def_myself.h"
#include "system_params.h"
#include "dc_servo.h"

/* Object form*/
typedef struct PID_t
{
	// PID parameters
	double	Kp, Ki, Kd;
	// Input and Output of PID controller
	double	v_setpoint, v_feedback , output;
	// Inside Variable
	double	pre_output;
	double	error, pre_error, pre2_error;
}PID_t;

/* Function Prototype */
void 	PID_Reset(PID_t *pid);
void 	PID_Compute(PID_t *pid, DcServo_t *motor);
void 	PID_Setpoint(PID_t *pid, double setpoint);
double	PID_GetSetpoint(PID_t *pid);
void 	PID_SetFactor(PID_t *pid, double kp, double ki, double kd);
void	PID_GetFactor(PID_t *pid, double *kp, double *ki, double *kd);

#endif /* INC_PID_MOTOR_H_ */
