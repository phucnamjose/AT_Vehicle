/*
 * step_motor.h
 *
 *  Created on: Nov 7, 2020
 *      Author: Dang Nam
 */

#ifndef INC_STEP_MOTOR_H_
#define INC_STEP_MOTOR_H_

#include "def_myself.h"


#define MAX_TIMER_COUNT			(10000)

typedef enum enum_Step{
	STEP_UP = 0,
	STEP_DOWN,
}enum_Step;

/* Object form*/
typedef struct Step_t
{
	int32_t		pulse_acc;
	double		postion_real; // radian
	int32_t		offset;
	uint8_t		limit_positive;
	uint8_t		limit_negative;
	int8_t		pulse_scan;
	uint8_t		Out_Dir_Invert;
	// Timer information
	TIM_TypeDef *TIM_PULSE;
	// Interrupte timer touch
	uint8_t		pulse_in_period;
	uint8_t		pulse_count_in_period;
}Step_t;

/* Function prototypes */
void	StepInit(Step_t *step, uint8_t out_dir_invert, int8_t pulse_scan, TIM_TypeDef *TIM_PULSE);
void	StepReadLimit(Step_t *step_up, Step_t *step_down);
uint8_t	StepWritePusle(Step_t *step_up, int8_t pulse_up, Step_t *step_down, int8_t pulse_down);
void	StepEnable(void);
void	StepDisable(void);
double	StepPulse2Angle(enum_Step step, int32_t pulse);
int32_t	StepAngle2Pulse(enum_Step step, double angle);

#endif /* INC_STEP_MOTOR_H_ */
