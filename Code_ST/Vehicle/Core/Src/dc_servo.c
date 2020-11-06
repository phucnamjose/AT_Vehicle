/*
 * dc_servo.c
 *
 *  Created on: Oct 25, 2020
 *      Author: Dang Nam
 */

#include "dc_servo.h"
#include "system_params.h"

/* Global variable*/
DcServo_t MR;
DcServo_t ML;

/* Implementation */

void DcServoInit(DcServo_t *motor,
				uint8_t out_dir_invert,
				uint8_t enc_dir_invert,
				TIM_TypeDef *TIM_PWM,
				TIM_TypeDef *TIM_ENC) {
	// Reset
	motor->Enc = 0;
	motor->current_v = 0;
	motor->out_percent = 0;
	// Save timer address
	motor->TIM_PWM = TIM_PWM;
	motor->TIM_ENC = TIM_ENC;
	// Direction
	motor->Out_Dir_Invert = out_dir_invert;
	motor->Enc_Dir_Invert = enc_dir_invert;
}

void DcVelUpdate(DcServo_t *motor) {
	// Previous
	motor->pre_pre_pre_v 	= motor->pre_pre_v;
	motor->pre_pre_v		= motor->pre_v;
	motor->pre_v			= motor->v;

	//motor->pre_v 	= motor->v;
	motor->Pre_Enc 	= motor->Enc;
	// Current
	motor->Enc		= motor->TIM_ENC->CNT;
	motor->Diff_Enc = motor->Enc - motor->Pre_Enc;
	// Check timer counter overflow
	if (motor->Diff_Enc > 30000)
		motor->Diff_Enc -= 0xFFFF;
	else if (motor->Diff_Enc < -30000)
		motor->Diff_Enc += 0xFFFF;
	// Direction
	if (motor->Enc_Dir_Invert)
		motor->Diff_Enc = motor->Diff_Enc*(-1);
	// Update velocity (round/min)
	motor->current_v=(((double)motor->Diff_Enc/ENCODER_RESOLUTION))/BASIC_PERIOD;
	//motor->current_v = 0.25*(motor->v + motor->pre_v + motor->pre_pre_v + motor->pre_pre_pre_v);
}

void DcExecuteOuput(DcServo_t *motor) {
	uint8_t 	dir = 0;
	uint16_t 	pulses;
	double 		out_percent;

	out_percent = motor->out_percent;
	// Direction
	if (motor->Out_Dir_Invert) {
		out_percent = out_percent*(-1);
	}
	if (out_percent < 0) {
		out_percent = -out_percent;
		dir = 1;
	}
	// Saturation
	if (out_percent > 0.6)
		out_percent = 0.6;
	// Convert duty to pulses time
	pulses = out_percent * PWM_RESOLUTION;
	// Write to timer
	if (dir == 0) {
		motor->TIM_PWM->CCR1 = pulses;
		motor->TIM_PWM->CCR2 = 0;
	} else {
		motor->TIM_PWM->CCR1 = 0;
		motor->TIM_PWM->CCR2 = pulses;
	}
}

double DcGetVel(DcServo_t *motor) {
	return motor->current_v;
}

void	DcSetOuput(DcServo_t *motor, double percent) {
	if (percent < 1.0 && percent > -1.0) {
		motor->out_percent = percent;
		DcExecuteOuput(motor);
	}
}

void	DcStopMotor(DcServo_t *motor) {
	motor->out_percent = 0;
	DcExecuteOuput(motor);
}
