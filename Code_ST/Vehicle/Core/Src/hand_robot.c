/*
 * hand_robot.c
 *
 *  Created on: Oct 31, 2020
 *      Author: Dang Nam
 */

#include "hand_robot.h"
#include "step_motor.h"
#include "main.h"
#include "system_params.h"

/* External variables */
extern Step_t	stepDown;
extern Step_t	stepUp;

/* Internal variables */
Hand_t	myHand;

/* Implementation */

void	Hand_Init(void) {
	myHand.state_scan = SCAN_STEP_INIT;
	myHand.state_hard = SCAN_HARD_UP;
	StepEnable();
}


uint8_t	Hand_Scan(void) {
	uint8_t ret;

	switch (myHand.state_scan) {
		case SCAN_STEP_INIT:
			myHand.state_scan = SCAN_STEP_INIT;
			myHand.state_hard = SCAN_HARD_UP;
			stepUp.pulse_acc	= 0;
			stepDown.pulse_acc	= 0;
			ret = FALSE;
			break;
		case SCAN_STEP_HARD:
			if (Hand_ScanHard())
				myHand.state_scan = SCAN_STEP_SOFT;
			ret = FALSE;
			break;
		case SCAN_STEP_SOFT:
			if (Hand_GoToSoftLim())
				myHand.state_scan = SCAN_STEP_FINISH;
			ret = FALSE;
			break;
		case SCAN_STEP_FINISH:
			ret = TRUE;
			break;
		default:
			break;
	}

	return ret;
}

uint8_t Hand_ScanHard(void) {
	uint8_t ret;

	StepReadLimit(&stepUp, &stepDown);
	switch (myHand.state_hard) {
		case SCAN_HARD_UP:
			// Rotate step up until touch the sensor
			if (stepUp.limit_negative == 0) {
				myHand.state_hard = SCAN_HARD_DOWN;
			} else {
				StepWritePusle(&stepUp, stepUp.pulse_scan, &stepDown, 0);
			}
			ret = FALSE;
			break;
		case SCAN_HARD_DOWN:
			// Rotate step down until touch the sensor
			if (stepDown.limit_negative == 0) {
				myHand.state_hard = SCAN_HARD_DONE;
			} else {
				StepWritePusle(&stepUp, 0, &stepDown, stepDown.pulse_scan);
			}
			ret = FALSE;
			break;
		case SCAN_HARD_DONE:
			// Save offset
			stepUp.offset 	= stepUp.pulse_acc;
			stepDown.offset = stepDown.pulse_acc;
			myHand.offset_up 	= HARD_LIM_UP_NEG - stepUp.offset*2*PI*RATIO_STEP_UP;
			myHand.offset_down 	= HARD_LIM_DOWN_NEG - stepDown.offset*2*PI*RATIO_STEP_DOWN;

			ret = TRUE;
			break;
		default:
			ret = FALSE;
			break;
	}
	return ret;
}

uint8_t Hand_GoToSoftLim(void) {
	int8_t	pulse_up, pulse_down;

	Hand_UpdateTruePositon();
	if (myHand.theta_up < SOFT_LIM_UP_NEG)
		pulse_up 	= stepUp.pulse_scan;
	if (myHand.theta_up < SOFT_LIM_DOWN_NEG)
		pulse_down 	= stepDown.pulse_scan;

	StepWritePusle(&stepUp, pulse_up, &stepDown, pulse_down);

	if (pulse_up == 0 && pulse_down == 0)
		return TRUE;// The end
	else
		return FALSE;
}

void 	Hand_Stop(void) {

}

void 	Hand_Up(void) {

}

void 	Hand_Down(void) {

}

void 	Hand_Left(void) {

}

void 	Hand_Right(void) {

}

void	Hand_UpdateTruePositon(void) {
	myHand.theta_up 	= myHand.offset_up + stepUp.pulse_acc*2.0*PI*RATIO_STEP_UP;
	myHand.theta_down 	= myHand.offset_down + stepUp.pulse_acc*2.0*PI*RATIO_STEP_DOWN;
}

void	Hand_ExcuteNewPosition(void) {

}
