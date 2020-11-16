/*
 * step_motor.c
 *
 *  Created on: Nov 7, 2020
 *      Author: Dang Nam
 */


#include "step_motor.h"
#include "main.h"

/* External variables */
extern TIM_HandleTypeDef htim5; // Step 1 on board
extern TIM_HandleTypeDef htim9; // Step 2 on board


/* Internal variables */
Step_t	stepDown; // Step 2 on board
Step_t	stepUp; // Step 1 on board

/* Implementation */

void	StepInit(Step_t *step, uint8_t out_dir_invert, int8_t pulse_scan, TIM_TypeDef *TIM_PULSE) {
	// Reset
	step->pulse_acc	 = 0;
	// Save timer address
	step->TIM_PULSE = TIM_PULSE;
	// Direction
	step->Out_Dir_Invert = out_dir_invert;
}

void	StepReadLimit(Step_t *step_up, Step_t *step_down) {
	step_up->limit_negative 	= HAL_GPIO_ReadPin(LS_UP_GPIO_Port, LS_UP_Pin);
	step_down->limit_negative 	= HAL_GPIO_ReadPin(LS_DOWN_L_GPIO_Port, LS_DOWN_R_Pin);
	step_down->limit_positive 	= HAL_GPIO_ReadPin(LS_DOWN_L_GPIO_Port, LS_DOWN_L_Pin);
}

uint8_t	StepWritePusle(Step_t *step_up, int8_t pulse_up, Step_t *step_down, int8_t pulse_down) {
	uint8_t pulse_up_abs, pulse_down_abs;
	uint8_t dir_up, dir_down;

	// Check timer stop
	if (__HAL_TIM_GET_FLAG(&htim5, TIM_CR1_CEN)) {
		while(1);
	}
	if (__HAL_TIM_GET_FLAG(&htim9, TIM_CR1_CEN)) {
		while(1);
	}

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
	step_up->pulse_in_period = pulse_up_abs;
	if (pulse_up_abs == 0) {
		step_up->TIM_PULSE->ARR = MAX_TIMER_COUNT - 1;
		step_up->TIM_PULSE->CCR1 = MAX_TIMER_COUNT;//Compare > AutoLoad=No pulse
	} else {
		uint16_t period_up = MAX_TIMER_COUNT/pulse_up_abs;
		uint16_t compare_up = period_up*7/10; // duty 70%
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
		HAL_GPIO_WritePin(STEP2_DIR_GPIO_Port, STEP2_DIR_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(STEP2_DIR_GPIO_Port, STEP2_DIR_Pin, GPIO_PIN_RESET);
	}
	// Check limit
	if (pulse_down_abs > LIM_PULSE_UP)
		return FALSE;
	// Set timer
	step_down->pulse_in_period = pulse_down_abs;
	if (pulse_down_abs == 0) {
		step_down->TIM_PULSE->ARR = MAX_TIMER_COUNT - 1;
		step_down->TIM_PULSE->CCR1 = MAX_TIMER_COUNT;
	} else {
		uint16_t period_down = MAX_TIMER_COUNT/pulse_down_abs;
		uint16_t compare_down = period_down*7/10; // duty 70%
		step_down->TIM_PULSE->ARR = period_down - 1;
		step_down->TIM_PULSE->CCR1 = compare_down;
	}
	step_down->pulse_count_in_period = 0;

	/* Turn on PWM */
	// Step 1 - up
	HAL_TIM_PWM_Start_IT(&htim5, TIM_CHANNEL_1);
	// Step 2 - down
	HAL_TIM_PWM_Start_IT(&htim9, TIM_CHANNEL_1);
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
