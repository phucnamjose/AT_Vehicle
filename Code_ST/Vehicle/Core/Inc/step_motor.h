/*
 * step_motor.h
 *
 *  Created on: Nov 7, 2020
 *      Author: Dang Nam
 */

#ifndef INC_STEP_MOTOR_H_
#define INC_STEP_MOTOR_H_

#include "def_myself.h"

#define LIM_PULSE_UP		50
#define LIM_PULSE_DOWN		50

#define	HARD_LIM_UP_NEG		(-0.1)
#define	HARD_LIM_DOWN_NEG	(-0.1)

#define	SOFT_LIM_UP_NEG		(-0.1)
#define	SOFT_LIM_UP_POS		(0.1)
#define	SOFT_LIM_DOWN_NEG	(-0.1)
#define	SOFT_LIM_DOWN_POS	(0.1)

#define RESOLUTION_STEP_UP		(200)
#define RESOLUTION_STEP_DOWN	(200)

#define MICRO_STEP_UP			(1)
#define MICRO_STEP_DOWN			(1)

#define MAX_TIMER_COUNT			(10000)

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

#endif /* INC_STEP_MOTOR_H_ */
