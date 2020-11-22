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
#include "stdlib.h"
#include "math.h"

/* External variables */
extern Step_t	stepDown;
extern Step_t	stepUp;

/* Internal variables */
Hand_t	myHand;
double	tableRectangle[4][2] = {	//	(1)<------------(4)
		{1.221730476, -1.62315204},	//	 |				 ^
		{1.221730476, -2.0},		//	 |	 Rectangle	 |
		{-1.221730476, -2.0},		//	 v				 |
		{-1.221730476, -1.62315204}	//	(2)------------>(3)
};

/* Implementation */

void	Hand_Init(void) {
	myHand.mode				= HAND_MODE_HOME;
	myHand.mode_next_after_home = HAND_MODE_MANUAL;
	myHand.flag_home_soft	= FALSE;
	myHand.type_manual 		= HAND_MAN_STOP;
	myHand.type_auto		= HAND_AUTO_RETANGLE;
	myHand.index_rectangle	= 0;
	myHand.speed			= 1;
	myHand.flag_error		= FALSE;
	myHand.action_up		= STEP_STOP;
	myHand.action_down		= STEP_STOP;
	myHand.action_up_pre	= STEP_STOP;
	myHand.action_down_pre	= STEP_STOP;

	myHand.is_scanned 		= FALSE;
	//myHand.is_scanned 		= TRUE;
	myHand.state_scan 		= SCAN_STEP_INIT;
	myHand.state_hard 		= SCAN_HARD_UP;
	StepEnable();
}

void	Hand_ChangeMode(enum_HandMode mode) {
	switch (mode) {
		case HAND_MODE_HOME:
			myHand.mode = mode;
			break;
		case HAND_MODE_MANUAL:
			myHand.mode = mode;
			break;
		case HAND_MODE_AUTO:
			myHand.mode = mode;
			break;
		default:
			break;
	}
}

void	Hand_ChangeSpeed(double percent) {
	if (percent >= 0 && percent <= 1)
		myHand.speed = percent;
}

void	Hand_Run(void) {
	// Check scan
	if (!Hand_IsScanned()) {
		Hand_Scan();
		return;
	} else {
		Hand_UpdateTruePositon();
	}
	// Run according to mode
	switch (myHand.mode) {
		case HAND_MODE_HOME:
			Hand_HomeRun();
			break;
		case HAND_MODE_MANUAL:
			Hand_ManualRun();
			break;
		case HAND_MODE_AUTO:
			Hand_AutoRun();
			break;
		default:
			break;
	}
}

uint8_t	Hand_Scan(void) {
	uint8_t ret;

	switch (myHand.state_scan) {
		case SCAN_STEP_INIT:
			myHand.state_scan = SCAN_STEP_HARD;
			myHand.state_hard = SCAN_HARD_UP;
			stepUp.pulse_acc	= 0;
			stepDown.pulse_acc	= 0;
			ret = FALSE;
			break;
		case SCAN_STEP_HARD:
			if (Hand_ScanHard())
				myHand.state_scan = SCAN_STEP_FINISH;
			ret = FALSE;
			break;
		case SCAN_STEP_FINISH:
			myHand.is_scanned = TRUE;
			myHand.mode = HAND_MODE_HOME;
			Hand_StartGoHome();
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
			if (stepUp.limit_positive) {
				myHand.state_hard = SCAN_HARD_DOWN;
			} else {
				StepWritePusle(&stepUp, stepUp.pulse_scan, &stepDown, 0);
			}
			ret = FALSE;
			break;
		case SCAN_HARD_DOWN:
			// Rotate step down until touch the sensor
			if (stepDown.limit_negative) {
				myHand.state_hard = SCAN_HARD_DONE;
			} else {
				StepWritePusle(&stepUp, 0, &stepDown, -stepDown.pulse_scan);
			}
			ret = FALSE;
			break;
		case SCAN_HARD_DONE:
			// Save offset
			stepUp.offset 	= stepUp.pulse_acc;
			stepDown.offset = stepDown.pulse_acc;
			myHand.offset_up 	= HARD_LIM_UP_POS
						- StepPulse2Angle(STEP_UP, stepUp.offset);
			myHand.offset_down 	= HARD_LIM_DOWN_NEG
						- StepPulse2Angle(STEP_DOWN, stepDown.offset);
			StepWritePusle(&stepUp, 0, &stepDown, 0);
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
	if (myHand.theta_up > SOFT_LIM_UP_POS)
		pulse_up 	= -stepUp.pulse_scan;
	if (myHand.theta_down < SOFT_LIM_DOWN_NEG)
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

uint8_t	Hand_ExcuteNewPosition(double theta_down, double theta_up) {
	int32_t	current_var0, current_var1;
	int32_t	next_var0, next_var1;
	int64_t		delta_var0, delta_var1;
	uint8_t 	result;

	current_var0	= StepAngle2Pulse(STEP_DOWN, myHand.theta_down);
	current_var1	= StepAngle2Pulse(STEP_UP, myHand.theta_up);
	next_var0		= StepAngle2Pulse(STEP_DOWN, theta_down);
	next_var1		= StepAngle2Pulse(STEP_UP, theta_up);
	delta_var0 = next_var0 - current_var0;
	delta_var1 = next_var1 - current_var1;

	if (abs(delta_var0) > LIM_PULSE_DOWN
		|| abs(delta_var1 > LIM_PULSE_UP)) {
		return FALSE;
	}

	result = StepWritePusle(&stepUp,
							(int8_t)delta_var1,
							&stepDown,
							(int8_t)delta_var0);

	return result;
}

void	Hand_AutoRun(void) {
	// Path Rectangle
	if (HAND_AUTO_RETANGLE == myHand.type_auto) {
		switch (myHand.state_auto) {
			case HAND_AUTO_INIT:
			{
				double goal_down, goal_up;
				goal_down 	= tableRectangle[myHand.index_rectangle][0];
				goal_up		= tableRectangle[myHand.index_rectangle][1];
				Hand_InitDuty(&(myHand.duty), goal_down, goal_up);
				myHand.state_auto = HAND_AUTO_RUNNING;
			}
				break;
			case HAND_AUTO_RUNNING:
				Hand_FollowDuty(&(myHand.duty));
				if (Hand_IsFinishedDuty(&(myHand.duty)))
					myHand.state_auto = HAND_AUTO_FINISH;
				break;
			case HAND_AUTO_FINISH:
				myHand.index_rectangle = (myHand.index_rectangle + 1) % 4;
				myHand.state_auto = HAND_AUTO_INIT;// Begin path again
				if (myHand.flag_home_soft) {
					Hand_StartGoHome();
					myHand.flag_home_soft = FALSE;
					myHand.index_rectangle = 0;
				}
				break;
			default:
				break;
		}
	// Path Circle
	} else if (HAND_AUTO_CIRCLE == myHand.type_auto) {
		switch (myHand.state_auto) {
			case HAND_AUTO_INIT:
				myHand.state_auto = HAND_AUTO_RUNNING;
				break;
			case HAND_AUTO_RUNNING:

				break;
			case HAND_AUTO_FINISH:

				break;
			default:
				break;
		}
	}
}

uint8_t	Hand_InitDuty(Hand_Duty_t *duty, double goal_down, double goal_up) {
	// Check goal theta


	myHand.duty.angle_start[0] = myHand.theta_down;
	myHand.duty.angle_start[1] = myHand.theta_up;

	double q[2];
	q[0] = goal_down - myHand.theta_down;
	q[1] = goal_up	- myHand.theta_up;

	double v_design[2];
	double a_design[2];
	v_design[0] = myHand.speed*MAX_SPEED_STEP_DOWN;
	v_design[1] = myHand.speed*MAX_SPEED_STEP_UP;
	a_design[0]	= v_design[0] / 1.0;
	a_design[1]	= v_design[1] / 1.0;
	// Init trajectory
	uint8_t status[2];
	for ( uint8_t i = 0; i < 2; i++) {
		status[i] = trajecInitLSPB(&(duty->trajec[i]),
								q[i],
								TRAJEC_INIT_QVA,
								v_design[i],
								a_design[i]);
	}
	// Choose total time
	duty->time_total = 0;
	for ( uint8_t i = 0; i < 2; i++) {
		if (duty->trajec[i].Tf > duty->time_total) {
			duty->time_total = duty->trajec[i].Tf;
		}
	}
	// Reinit with total time
	for ( uint8_t i = 0; i < 2; i++) {
		duty->trajec[i].Tf = duty->time_total;
		status[i] = trajecInitLSPB(&(duty->trajec[i]),
								q[i],
								TRAJEC_INIT_QVT,
								v_design[i],
								a_design[i]);
	}
	// Check status
	for ( uint8_t i = 0; i < 2; i++) {
		if (!status[i])
			return FALSE;
	}
	// Ok, reset run time
	duty->time_run = 0;
	duty->is_finished = FALSE;

	return TRUE;
}

uint8_t Hand_FollowDuty(Hand_Duty_t *duty) {

	double status[2];
	double dir[2];
	double s[2];
	//double v[2];
	double theta[2];

	duty->time_run += BASIC_PERIOD;
	// Compute new theta with current time
	for ( uint8_t i = 0; i < 2; i++) {
		status[i] 	= trajecFlowLSPB(&(duty->trajec[i]), duty->time_run);
		dir[i]		= duty->trajec[i].dir;
		s[i]		= duty->trajec[i].s_current;
		//v[i]		= duty->trajec[i].v_current;
		if (!status[i])
			return FALSE;
	}
	// Ok
	for (uint8_t i = 0; i < 2; i++) {
		theta[i] = duty->angle_start[i] + s[i]*dir[i];
	}
	uint8_t ret = Hand_ExcuteNewPosition(theta[0], theta[1]);
	// Check finish
	if (duty->time_run > duty->time_total)
		duty->is_finished = TRUE;

	return ret;
}

uint8_t	Hand_IsFinishedDuty(Hand_Duty_t *duty) {
	return duty->is_finished;
}

void	Hand_UpdateSpeedBound(void) {
	double dis_up_pos, dis_up_neg;
	double dis_down_pos, dis_down_neg;
	// Compute distance from current angle to soft limit
	dis_up_pos 		= SOFT_LIM_UP_POS - myHand.theta_up;
	dis_up_neg 		= SOFT_LIM_UP_NEG - myHand.theta_up;
	dis_down_pos 	= SOFT_LIM_DOWN_POS - myHand.theta_down;
	dis_down_neg 	= SOFT_LIM_DOWN_NEG - myHand.theta_down;

	double dis_max_up;
	double dis_max_down;
	dis_max_up = 24.5*myHand.speed*MAX_SPEED_STEP_UP*BASIC_PERIOD;
	dis_max_down = 24.5*myHand.speed*MAX_SPEED_STEP_DOWN*BASIC_PERIOD;
	// Bound up positive
	if (fabs(dis_up_pos) >= dis_max_up) {
		myHand.speed_bound_up_pos = myHand.speed*MAX_SPEED_STEP_UP;
	} else {
		double test = 0;
		for (int i = 1; i <= 50; i++) {
			test += (0.02*i)*myHand.speed*MAX_SPEED_STEP_UP*BASIC_PERIOD;
			if (test > fabs(dis_up_pos)) {
				myHand.speed_bound_up_pos = (i-1)*0.02*myHand.speed*MAX_SPEED_STEP_UP;
				break;
			}
		}
	}
	if (myHand.theta_up > SOFT_LIM_UP_POS)
		myHand.speed_bound_up_pos = 0;

	// Bound up neg
	if (fabs(dis_up_neg) >= dis_max_up) {
		myHand.speed_bound_up_neg = -myHand.speed*MAX_SPEED_STEP_UP;
	} else {
		double test = 0;
		for (int i = 1; i <= 50; i++) {
			test += (0.02*i)*myHand.speed*MAX_SPEED_STEP_UP*BASIC_PERIOD;
			if (test > fabs(dis_up_neg)) {
				myHand.speed_bound_up_neg = -(i-1)*0.02*myHand.speed*MAX_SPEED_STEP_UP;
				break;
			}
		}
	}
	// Bound down positive
	if (fabs(dis_down_pos) >= dis_max_down) {
		myHand.speed_bound_down_pos = myHand.speed*MAX_SPEED_STEP_DOWN;
	} else {
		double test = 0;
		for (int i = 1; i <= 50; i++) {
			test += (0.02*i)*myHand.speed*MAX_SPEED_STEP_DOWN*BASIC_PERIOD;
			if (test > fabs(dis_down_pos)) {
				myHand.speed_bound_down_pos = (i-1)*0.02*myHand.speed*MAX_SPEED_STEP_DOWN;
				break;
			}
		}
	}
	if (myHand.theta_down > SOFT_LIM_DOWN_POS)
			myHand.speed_bound_down_pos = 0;

	// Bound up neg
	if (fabs(dis_down_neg) >= dis_max_down) {
		myHand.speed_bound_down_neg = -myHand.speed*MAX_SPEED_STEP_DOWN;
	} else {
		double test = 0;
		for (int i = 1; i <= 50; i++) {
			test += (0.02*i)*myHand.speed*MAX_SPEED_STEP_DOWN*BASIC_PERIOD;
			if (test > fabs(dis_down_neg)) {
				myHand.speed_bound_down_neg = -(i-1)*0.02*myHand.speed*MAX_SPEED_STEP_DOWN;
				break;
			}
		}
	}
	if (myHand.theta_down < SOFT_LIM_DOWN_NEG)
			myHand.speed_bound_down_neg = 0;
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

void	Hand_ManualRun(void) {
	// Bound speed when it near bound
	Hand_UpdateSpeedBound();
	// Two joint work independently
	double angle_up, angle_down;
	double position_up, position_down;
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
			double vel_set, max_speed, bound;
			max_speed = myHand.speed*MAX_SPEED_STEP_UP;
			vel_set = myHand.speed_up_current + 0.02*max_speed;
			bound	= myHand.speed_bound_up_pos;
			vel_set = (vel_set > bound) ? bound : vel_set;
			myHand.speed_up_current = vel_set;
		}
			break;
		case STEP_GO_NEGATIVE:
		{
			double vel_set, max_speed, bound;
			max_speed = myHand.speed*MAX_SPEED_STEP_UP;
			vel_set = myHand.speed_up_current - 0.02*max_speed;
			bound	= myHand.speed_bound_up_neg;
			vel_set = (vel_set < bound) ? bound : vel_set;
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
			double vel_set, max_speed, bound;
			max_speed = myHand.speed*MAX_SPEED_STEP_DOWN;
			vel_set = myHand.speed_down_current + 0.02*max_speed;
			bound 	= myHand.speed_bound_down_pos;
			vel_set = (vel_set > bound) ? bound : vel_set;
			myHand.speed_down_current = vel_set;
		}
			break;
		case STEP_GO_NEGATIVE:
		{
			double vel_set, max_speed, bound;
			max_speed = myHand.speed*MAX_SPEED_STEP_DOWN;
			vel_set = myHand.speed_down_current - 0.02*max_speed;
			bound	= myHand.speed_bound_down_neg;
			vel_set = (vel_set < bound) ? bound : vel_set;
			myHand.speed_down_current = vel_set;
		}
			break;
		default:
			break;
	}
	// Compute next position
	angle_up 	= (myHand.speed_up_current*BASIC_PERIOD);
	angle_down 	= (myHand.speed_down_current*BASIC_PERIOD);
	position_up 	= myHand.theta_up + angle_up;
	position_down 	= myHand.theta_down + angle_down;

	// Execute according next position
	Hand_ExcuteNewPosition(position_down, position_up);

	// Check timeout
	if (myHand.type_manual > HAND_MAN_STOP) {
		myHand.timeout_hand_manual += 0.01;
		if (myHand.timeout_hand_manual > TIME_LIMIT_HAND_REFRESH) {
			Hand_Stop();
		}
	}

}

void	Hand_StartGoHome(void) {
	Hand_ChangeMode(HAND_MODE_HOME);
	myHand.state_homing = HOME_STATE_INIT;
}

void	Hand_HomeRun(void) {
	switch (myHand.state_homing) {
		case HOME_STATE_INIT:
			Hand_InitDuty(&(myHand.duty), ANGLE_HOME_DOWN, ANGLE_HOME_UP);
			myHand.state_homing = HOME_STATE_FOLLOW;

			break;
		case HOME_STATE_FOLLOW:
			Hand_FollowDuty(&(myHand.duty));
			if (Hand_IsFinishedDuty(&(myHand.duty)))
				myHand.state_homing = HOME_STATE_FINISH;
			break;
		case HOME_STATE_FINISH:
			if (myHand.flag_home_soft) {
				Hand_StartGoHome();
				myHand.flag_home_soft = FALSE;
			}
			myHand.mode = myHand.mode_next_after_home;
			break;
		default:
			break;
	}
}
