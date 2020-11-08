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

// TUNED
#define PID_DEFAULT_RIGHT_KP	0.37987
#define PID_DEFAULT_RIGHT_KI	7.0512
#define PID_DEFAULT_RIGHT_KD	0.0037946
#define PID_DEFAULT_LEFT_KP		0.3404
#define PID_DEFAULT_LEFT_KI		6.3148
#define PID_DEFAULT_LEFT_KD		0.0027876


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
	// Count
	int32_t count;
	uint8_t run_black_box;
}PID_t;

/* Function Prototype */
void 	PID_Reset(PID_t *pid);
void 	PID_Compute(PID_t *pid, DcServo_t *motor);
void 	PID_Setpoint(PID_t *pid, double setpoint);
double	PID_GetSetpoint(PID_t *pid);
void 	PID_SetFactor(PID_t *pid, double kp, double ki, double kd);
void	PID_GetFactor(PID_t *pid, double *kp, double *ki, double *kd);
void	PID_TestSquareWave(PID_t *pid, DcServo_t *motor);
double	PID_GetOutput(PID_t *pid);
int32_t	PID_GetCount(PID_t *pid);

void	PID_StartBlackBox(PID_t *pid);
uint8_t	PID_RunBlackBox(PID_t *pid, DcServo_t *motor);

#endif /* INC_PID_MOTOR_H_ */
