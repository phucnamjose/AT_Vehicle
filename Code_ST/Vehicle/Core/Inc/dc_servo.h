/*
 * dc_servo.h
 *
 *  Created on: Oct 25, 2020
 *      Author: Dang Nam
 */

#ifndef INC_DC_SERVO_H_
#define INC_DC_SERVO_H_

#include <tim.h>
#include "def_myself.h"

#define ENCODER_RESOLUTION	(600*4)
#define PWM_RESOLUTION		(200)
#define VOLT_MOTION			(12)

/* Object form*/
typedef struct DcServo_t
{
	// Encoder	(pulse)
	uint16_t	Enc;
	uint16_t	Pre_Enc;
	int32_t		Diff_Enc;
	uint8_t		Enc_Dir_Invert;
	// Velocity (round/min)
	double		current_v;
	double		v;
	double		pre_v;
	double		pre_pre_v;
	double		pre_pre_pre_v;
	// Output Percent (%)
	double		out_percent;
	uint8_t		Out_Dir_Invert;
	// Timer information
	TIM_TypeDef	*TIM_PWM;
	TIM_TypeDef *TIM_ENC;
}DcServo_t;

/* Function Prototype */
void 	DcServoInit(DcServo_t *motor,
					uint8_t out_dir_invert,
					uint8_t enc_dir_invert,
					TIM_TypeDef *TIM_PWM,
					TIM_TypeDef *TIM_ENC);
void 	DcVelUpdate(DcServo_t *motor);
void 	DcExecuteOuput(DcServo_t *motor);
double	DcGetVel(DcServo_t *motor);
void	DcSetOuput(DcServo_t *motor, double percent);
void	DcStopMotor(DcServo_t *motor);

#endif /* INC_DC_SERVO_H_ */
