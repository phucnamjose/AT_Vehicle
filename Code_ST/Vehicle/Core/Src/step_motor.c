/*
 * step_motor.c
 *
 *  Created on: Nov 7, 2020
 *      Author: Dang Nam
 */


#include "step_motor.h"
#include "system_params.h"
#include "main.h"
#include "math.h"

/* External variables */
extern TIM_HandleTypeDef htim5; // Step 1 on board
extern TIM_HandleTypeDef htim9; // Step 2 on board


/* Internal variables */
Step_t	stepDown; // Step 2 on board
Step_t	stepUp; // Step 1 on board

/* Implementation */

void	StepInit(Step_t *step,
				uint8_t out_dir_invert,
				int8_t pulse_scan,
				TIM_TypeDef *TIM_PULSE) {
	// Reset
	step->pulse_acc	 = 0;
	step->pulse_in_period = 0;
	step->pulse_count_in_period = 0;
	// Scan
	step->pulse_scan = pulse_scan;
	// Save timer address
	step->TIM_PULSE = TIM_PULSE;
	// Direction
	step->Out_Dir_Invert = out_dir_invert;
}

void	StepReadLimit(Step_t *step_up, Step_t *step_down) {
	uint8_t level_up_pos, level_down_neg, level_down_pos;
	level_up_pos = HAL_GPIO_ReadPin(LS_UP_GPIO_Port, LS_UP_Pin);
	level_down_neg = HAL_GPIO_ReadPin(LS_DOWN_L_GPIO_Port, LS_DOWN_L_Pin);
	level_down_pos = HAL_GPIO_ReadPin(LS_DOWN_R_GPIO_Port, LS_DOWN_R_Pin);

	step_up->limit_positive 	= (LEVEL_ACTIVE_LS_UP_POS == level_up_pos);
	step_down->limit_negative 	= (LEVEL_ACTIVE_LS_DOWN_NEG == level_down_neg);
	step_down->limit_positive 	= (LEVEL_ACTIVE_LS_DOWN_POS == level_down_pos);
}

uint8_t	StepWritePusle(Step_t *step_up,
						int8_t pulse_up,
						Step_t *step_down,
						int8_t pulse_down) {
	uint8_t pulse_up_abs, pulse_down_abs;
	uint8_t dir_up, dir_down;

	/* Turn off PWM */
	HAL_TIM_Base_Stop(&htim5);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	HAL_TIM_Base_Stop(&htim9);
	__HAL_TIM_SET_COUNTER(&htim9, 0);

	/* Step up */
	// Direction
	if (pulse_up < 0) {
		pulse_up_abs = -pulse_up;
		dir_up		 = 1;
	} else {
		pulse_up_abs = pulse_up;
		dir_up		 = 0;
	}
	if (dir_up ^ (step_up->Out_Dir_Invert)) {
		HAL_GPIO_WritePin(STEP1_DIR_GPIO_Port, STEP1_DIR_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(STEP1_DIR_GPIO_Port, STEP1_DIR_Pin, GPIO_PIN_RESET);
	}
	// Check limit
	if (pulse_up_abs > LIM_PULSE_UP)
		return FALSE;
	// Set timer
	step_up->pulse_in_period = pulse_up_abs + 1;
	if (pulse_up_abs == 0) {
		//Compare > AutoLoad=No pulse
		step_up->TIM_PULSE->ARR = MAX_TIMER_COUNT - 1;
		step_up->TIM_PULSE->CCR1 = MAX_TIMER_COUNT + 1;
	} else {
		uint16_t period_up = MAX_TIMER_COUNT/pulse_up_abs;
		uint16_t compare_up = period_up*5/10; // duty 50%
		step_up->TIM_PULSE->ARR = period_up - 1;
		step_up->TIM_PULSE->CCR1 = compare_up;
	}
	step_up->pulse_count_in_period = 0;

	/* Step down */
	// Direction
	if (pulse_down < 0) {
		pulse_down_abs		= -pulse_down;
		dir_down			= 1;
	} else {
		pulse_down_abs		= pulse_down;
		dir_down		 	= 0;
	}
	if (dir_down^(step_down->Out_Dir_Invert)) {
		HAL_GPIO_WritePin(STEP2_DIR_GPIO_Port, STEP2_DIR_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(STEP2_DIR_GPIO_Port, STEP2_DIR_Pin, GPIO_PIN_SET);
	}
	// Check limit
	if (pulse_down_abs > LIM_PULSE_UP)
		return FALSE;
	// Set timer
	step_down->pulse_in_period = pulse_down_abs + 1;
	if (pulse_down_abs == 0) {
		step_down->TIM_PULSE->ARR = MAX_TIMER_COUNT - 1;
		step_down->TIM_PULSE->CCR1 = MAX_TIMER_COUNT + 1;
	} else {
		uint16_t period_down = MAX_TIMER_COUNT/pulse_down_abs;
		uint16_t compare_down = period_down*5/10; // duty 50%
		step_down->TIM_PULSE->ARR = period_down - 1;
		step_down->TIM_PULSE->CCR1 = compare_down;
	}
	step_down->pulse_count_in_period = 0;

	/* Turn on PWM */
	// Step 1 - up
	HAL_TIM_Base_Start(&htim5);
	// Step 2 - downs
	HAL_TIM_Base_Start(&htim9);
	// Until enough interrupt time == pulses

	// Accumulaate
	step_up->pulse_acc	 += pulse_up;
	step_down->pulse_acc += pulse_down;

	return TRUE;
}

void	StepEnable(void) {
	HAL_GPIO_WritePin(STEP1_EN_GPIO_Port, STEP1_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEP2_EN_GPIO_Port, STEP2_EN_Pin, GPIO_PIN_RESET);
}

void	StepDisable(void) {
	HAL_GPIO_WritePin(STEP1_EN_GPIO_Port, STEP1_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP2_EN_GPIO_Port, STEP2_EN_Pin, GPIO_PIN_SET);
}

double	StepPulse2Angle(enum_Step step, int32_t pulse) {
	double angle = 0;
	if (STEP_UP == step) {
		angle = (pulse/(RESOL_STEP_UP*MICRO_STEP_UP/RATIO_STEP_UP)*2*PI);
	} else if (STEP_DOWN == step) {
		angle = (pulse/(RESOL_STEP_DOWN*MICRO_STEP_DOWN/RATIO_STEP_DOWN)*2*PI);
	}

	return angle;
}

int32_t	StepAngle2Pulse(enum_Step step, double angle) {
	int32_t pulse = 0;
	if (STEP_UP == step) {
		pulse = round((angle*(RESOL_STEP_UP*MICRO_STEP_UP/RATIO_STEP_UP)/(2*PI)));
	} else if (STEP_DOWN == step) {
		pulse = round((angle*(RESOL_STEP_DOWN*MICRO_STEP_DOWN/RATIO_STEP_DOWN)/(2*PI)));
	}

	return pulse;
}
