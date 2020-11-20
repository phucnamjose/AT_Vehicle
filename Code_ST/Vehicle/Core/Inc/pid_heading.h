/*
 * pid_heading.h
 *
 *  Created on: Nov 11, 2020
 *      Author: Dang Nam
 */

#ifndef INC_PID_HEADING_H_
#define INC_PID_HEADING_H_

#include "def_myself.h"

// TUNED
#define HEAD_PERIOD				(0.05)

#define HEAD_PID_DEFAULT_KP		(0.514)
#define HEAD_PID_DEFAULT_KI		(0.0)
#define HEAD_PID_DEFAULT_KD		(0.0128)

/* Object form*/
typedef struct HeadPID_t
{
	// PID parameters
	double	Kp, Ki, Kd;
	// Input and Output of PID controller
	double	setpoint, feedback , output;
	double	output_each;
	// Inside Variable
	double	pre_output;
	double	error, pre_error, pre2_error;
	double	saturation;
	// Count
	int32_t count;
	uint8_t run_black_box;
}HeadPID_t;

/* Function Prototype */
void	Head_PID_Init();
void 	Head_PID_Reset(HeadPID_t *pid);
void 	Head_PID_SetFactor(HeadPID_t *pid, double kp, double ki, double kd);
void	Head_PID_UpdateFeedback(HeadPID_t *pid, double feedback);
void	Head_PID_Compute(HeadPID_t *pid);
void	Head_PID_StartBlackBox(HeadPID_t *pid);
uint8_t	Head_PID_RunBlackBox(HeadPID_t *pid);
void	Head_PID_SetOuput(HeadPID_t *pid, double vel);
void	Head_PID_SetPoint(HeadPID_t *pid, double angle);
double	Head_PID_GetOutput(HeadPID_t *pid);
double	Head_PID_GetOutputEach(HeadPID_t *pid);
void	Head_PID_SetSaturation(HeadPID_t *pid, double thresh);
int32_t	Head_PID_GetCount(HeadPID_t *pid);

#endif /* INC_PID_HEADING_H_ */
