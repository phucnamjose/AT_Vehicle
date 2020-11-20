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

uint32_t count = 0;

/* Implementation */

void	Hand_Init(void) {
	myHand.mode				= HAND_MODE_MANUAL;
	myHand.type_manual 		= HAND_MAN_STOP;
	myHand.type_auto		= HAND_AUTO_IDLE;
	myHand.speed			= 1;
	myHand.flag_error		= FALSE;
	myHand.action_up		= STEP_STOP;
	myHand.action_down		= STEP_STOP;
	myHand.action_up_pre	= STEP_STOP;
	myHand.action_down_pre	= STEP_STOP;

	//myHand.is_scanned 		= FALSE;
	myHand.is_scanned 		= TRUE;
	myHand.state_scan 		= SCAN_STEP_INIT;
	myHand.state_hard 		= SCAN_HARD_UP;
	StepEnable();
}

void	Hand_ChangeMode(enum_HandMode mode) {
	switch (mode) {
		case HAND_MODE_MANUAL:
			myHand.mode = mode;
			// Stop current action
			break;
		case HAND_MODE_AUTO:
			myHand.mode = mode;
			// Stop current action
			break;
		default:
			break;
	}
}

void	Hand_ChangeSpeed(double percent) {
	if (percent >= 0 && percent <= 1)
		myHand.speed = percent;
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
			myHand.offset_up 	= HARD_LIM_UP_NEG
						- StepPulse2Angle(STEP_UP, stepUp.offset);
			myHand.offset_down 	= HARD_LIM_DOWN_NEG
						- StepPulse2Angle(STEP_DOWN, stepDown.offset);

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

	pulse_up 	= 0;
	pulse_down	= 0;
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

uint8_t	Hand_IsScanned(void) {
	return myHand.is_scanned;
}

void	Hand_UpdateTruePositon(void) {
	myHand.theta_up 	= myHand.offset_up
			+ StepPulse2Angle(STEP_UP, stepUp.pulse_acc);
	myHand.theta_down 	= myHand.offset_down
			+ StepPulse2Angle(STEP_DOWN, stepDown.pulse_acc);
}

void	Hand_ExcuteNewPosition(void) {

}

void	Hand_AutoRunState(void) {
	switch (myHand.state_auto) {
		case HAND_AUTO_IDLE:
			// Do nothing
			__NOP();
			break;
		case HAND_AUTO_INIT:

			break;
		case HAND_AUTO_RUNNING:

			break;
		case HAND_AUTO_FINISH:

			break;
		default:
			break;
	}
}

void 	Hand_Stop(void) {
	myHand.type_manual = HAND_MAN_STOP;
	myHand.timeout_hand_manual = 0;
	myHand.action_up_pre = myHand.action_up;
	myHand.action_down_pre = myHand.action_down;
	myHand.action_up	= STEP_STOP;
	myHand.action_down	= STEP_STOP;
}

void 	Hand_Up(void) {
	myHand.type_manual = HAND_MAN_UP;
	myHand.timeout_hand_manual = 0;
	myHand.action_up	= STEP_GO_POSITIVE;
	myHand.action_down_pre = myHand.action_down;
	myHand.action_down	= STEP_STOP;
}

void 	Hand_Down(void) {
	myHand.type_manual = HAND_MAN_DOWN;
	myHand.timeout_hand_manual = 0;
	myHand.action_up	= STEP_GO_NEGATIVE;
	myHand.action_down_pre = myHand.action_down;
	myHand.action_down	= STEP_STOP;
}

void 	Hand_Left(void) {
	myHand.type_manual = HAND_MAN_LEFT;
	myHand.timeout_hand_manual = 0;
	myHand.action_up_pre = myHand.action_up;
	myHand.action_up	= STEP_STOP;
	myHand.action_down	= STEP_GO_POSITIVE;
}

void 	Hand_Right(void) {
	myHand.type_manual = HAND_MAN_RIGHT;
	myHand.timeout_hand_manual = 0;
	myHand.action_up_pre = myHand.action_up;
	myHand.action_up	= STEP_STOP;
	myHand.action_down	= STEP_GO_NEGATIVE;
}

void	Hand_GoHome(void) {

}

void	Hand_ManualRun(void) {
//	count = count + 10;
//	if (count < 4000) {
//		StepWritePusle(&stepUp, 0, &stepDown, 10);
//	} else if (count < 8000) {
//		StepWritePusle(&stepUp, 0, &stepDown, -10);
//	} else {
//		count = 0;
//	}

	// Check scan yet
	if (!Hand_IsScanned()) {
		return;
	}

	int8_t pulse_up, pulse_down;
	double angle_up, angle_down;
	double position_up, position_down;
	// Two joint work independently
	// Step up
	switch (myHand.action_up) {
		case STEP_STOP:
			if (STEP_GO_POSITIVE == myHand.action_up_pre) {
				double vel_set, max_speed;
				max_speed = myHand.speed*MAX_SPEED_STEP_UP;
				vel_set = myHand.speed_up_current - 0.02*max_speed;
				vel_set = (vel_set < 0) ? 0 : vel_set;
				myHand.speed_up_current = vel_set;
			} else if (STEP_GO_NEGATIVE == myHand.action_up_pre) {
				double vel_set, max_speed;
				max_speed = myHand.speed*MAX_SPEED_STEP_UP;
				vel_set = myHand.speed_up_current + 0.02*max_speed;
				vel_set = (vel_set > 0) ? 0 : vel_set;
				myHand.speed_up_current = vel_set;

			} else {
				myHand.speed_up_current = 0;
			}
			break;
		case STEP_GO_POSITIVE:
		{
			double vel_set, max_speed;
			max_speed = myHand.speed*MAX_SPEED_STEP_UP;
			vel_set = myHand.speed_up_current + 0.02*max_speed;
			vel_set = (vel_set > max_speed) ? max_speed : vel_set;
			myHand.speed_up_current = vel_set;
		}
			break;
		case STEP_GO_NEGATIVE:
		{
			double vel_set, max_speed;
			max_speed = myHand.speed*MAX_SPEED_STEP_UP;
			vel_set = myHand.speed_up_current - 0.02*max_speed;
			vel_set = (vel_set < (-max_speed)) ? -max_speed : vel_set;
			myHand.speed_up_current = vel_set;
		}
			break;
		default:
			break;
	}
	// Step down
	switch (myHand.action_down) {
		case STEP_STOP:
			if (STEP_GO_POSITIVE == myHand.action_down_pre) {
				double vel_set, max_speed;
				max_speed = myHand.speed*MAX_SPEED_STEP_DOWN;
				vel_set = myHand.speed_down_current - 0.02*max_speed;
				vel_set = (vel_set < 0) ? 0 : vel_set;
				myHand.speed_down_current = vel_set;
			} else if (STEP_GO_NEGATIVE == myHand.action_down_pre) {
				double vel_set, max_speed;
				max_speed = myHand.speed*MAX_SPEED_STEP_DOWN;
				vel_set = myHand.speed_down_current + 0.02*max_speed;
				vel_set = (vel_set > 0) ? 0 : vel_set;
				myHand.speed_down_current = vel_set;

			} else {
				myHand.speed_down_current = 0;
			}
			break;
		case STEP_GO_POSITIVE:
		{
			double vel_set, max_speed;
			max_speed = myHand.speed*MAX_SPEED_STEP_DOWN;
			vel_set = myHand.speed_down_current + 0.02*max_speed;
			vel_set = (vel_set > max_speed) ? max_speed : vel_set;
			myHand.speed_down_current = vel_set;
		}
			break;
		case STEP_GO_NEGATIVE:
		{
			double vel_set, max_speed;
			max_speed = myHand.speed*MAX_SPEED_STEP_DOWN;
			vel_set = myHand.speed_down_current - 0.02*max_speed;
			vel_set = (vel_set < (-max_speed)) ? -max_speed : vel_set;
			myHand.speed_down_current = vel_set;
		}
			break;
		default:
			break;
	}
	// Convert to pulse per 10ms
	angle_up 	= (myHand.speed_up_current*BASIC_PERIOD);
	pulse_up 	= StepAngle2Pulse(STEP_UP, angle_up);
	angle_down 	= (myHand.speed_down_current*BASIC_PERIOD);
	pulse_down 	= StepAngle2Pulse(STEP_DOWN, angle_down);
	// Check limit range to stop


	// Execute according current speed
	StepWritePusle(&stepUp, pulse_up, &stepDown, pulse_down);

	// Check timeout
	if (myHand.type_manual > HAND_MAN_STOP) {
		myHand.timeout_hand_manual += 0.01;
		if (myHand.timeout_hand_manual > TIME_LIMIT_HAND_REFRESH) {
			Hand_Stop();
		}
	}

}
