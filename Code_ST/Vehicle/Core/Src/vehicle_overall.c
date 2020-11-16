/*
 * vehicle_overall.c
 *
 *  Created on: Oct 31, 2020
 *      Author: Dang Nam
 */

#include "vehicle_overall.h"
#include "pid_motor.h"
#include "fuzzy_controller.h"
#include "stanley_controller.h"
#include "arduino.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"
#include "def_myself.h"
#include "dc_servo.h"



/* External Variables */
extern DcServo_t 	MR;
extern DcServo_t 	ML;
extern PID_t		pid_MR;
extern PID_t		pid_ML;
extern Fuzzy_t     	myFuzzy;
extern Stanley_t  	myStanley;

/* Internal Variables */
Vehicle_t 		myVehicle;


void	Vehicle_Init() {
	myVehicle.mode_vehicle 	= MODE_MANUAL;
	myVehicle.move_manual	= STOP_VEHICLE;
	myVehicle.time_out_move_manual 	= 0;
	myVehicle.miss_lidar_count		= 0;
	myVehicle.speed_auto			= 0.15; // m/s
	Fuzzy_Init();
	Stanley_Init(&myStanley);
}


void Vehicle_ChangeMode(enum_ModeVehicle mode) {
	switch (mode) {
		case MODE_AUTO:
			myVehicle.mode_vehicle = mode;
			break;
		case MODE_MANUAL:
			myVehicle.mode_vehicle = mode;
			break;
		default:
			break;
	}
}


void	Vehicle_ChangeSpeed(double speed) {
	myVehicle.speed_manual = speed;
}

void Vehicle_EstimatePosition(uint8_t has_lidar) {
	double x_center, y_center, heading;

	if (has_lidar) {
		// Note: Yaw(Lidar) != Yaw(Vehicle -- heading angle)
		heading = Pi_To_Pi(myVehicle.position_lidar.yaw + HEADING_OFFSET_LIDAR);
		// Center is in the middle 2 wheel
		x_center = myVehicle.position_lidar.x + DIS_LIDAR_FRONT_WHEEL*cos(heading);
		y_center = myVehicle.position_lidar.y + DIS_LIDAR_FRONT_WHEEL*sin(heading);
		// Update center
		myVehicle.position_center_veh.x 	= x_center;
		myVehicle.position_center_veh.y 	= y_center;
		myVehicle.position_center_veh.yaw 	= heading;
		// Update base position for odometry method
		memcpy(&(myVehicle.position_odometry),
			&(myVehicle.position_center_veh), sizeof(Position_t));
		myVehicle.miss_lidar_count = 0;
	} else {
		// Use result from odometry
		myVehicle.miss_lidar_count++;
		memcpy(&(myVehicle.position_center_veh),
			&(myVehicle.position_odometry), sizeof(Position_t));
	}
}

void	Vehicle_Odometry() {
	double delta_left, delta_right, delta_yaw;
	double x_new, y_new, yaw_new;
	double yaw_old;

	yaw_old = myVehicle.position_odometry.yaw;

	delta_left	= RPS2MPS(DcGetVel(&ML))*BASIC_PERIOD;
	delta_right	= RPS2MPS(DcGetVel(&MR))*BASIC_PERIOD;
	delta_yaw 	= (delta_right - delta_left)/(DIS_TWO_WHEELS);// DIS = 2L in formula


	x_new 	= myVehicle.position_odometry.x
				+ 0.5*(delta_left + delta_right)*cos(yaw_old+delta_yaw/2);
	y_new	= myVehicle.position_odometry.y
				+ 0.5*(delta_left + delta_right)*sin(yaw_old+delta_yaw/2);

	yaw_new	= Pi_To_Pi(yaw_old + delta_yaw);// [-Pi; Pi]
	// Update
	myVehicle.position_odometry.x 	= x_new;
	myVehicle.position_odometry.y 	= y_new;
	myVehicle.position_odometry.yaw = yaw_new;
}

void Vehicle_AutoDrive(void) {
	double v_left, v_right;
	double v_vehicle;
	double x_current, y_current, yaw_current;
	double v_long_left, v_long_right;

	x_current 	= myVehicle.position_center_veh.x;
	y_current	= myVehicle.position_center_veh.y;
	yaw_current	= myVehicle.position_center_veh.yaw;

	v_left		= RPS2MPS(DcGetVel(&ML));
	v_right		= RPS2MPS(DcGetVel(&MR));
	v_vehicle	= 0.5*(v_left + v_right);
	// Stanley Controller
	Stanley_Follow(&myStanley, x_current, y_current, yaw_current, v_vehicle);
	// Fuzzy Controller
	myFuzzy.Set_Angle 	= Pi_To_Pi(yaw_current + myStanley.Delta_Angle);
	myFuzzy.Angle		= yaw_current;
	Fuzzy_UpdateInput(&myFuzzy);
	myFuzzy.Fuzzy_Out = Fuzzy_Defuzzification_Max_Min(myFuzzy.Fuzzy_Error, myFuzzy.Fuzzy_Error_dot);
	// Update setpoint
	if (myFuzzy.Fuzzy_Out >= 0) {
		// Turn left
		v_long_right 	= (1 - fabs(myFuzzy.Fuzzy_Out))*myVehicle.speed_auto;// (meter/second)
		v_long_left		= (1 + fabs(myFuzzy.Fuzzy_Out))*myVehicle.speed_auto;
	} else {
		// Turn right
		v_long_right 	= (1 + fabs(myFuzzy.Fuzzy_Out))*myVehicle.speed_auto;
		v_long_left		= (1 - fabs(myFuzzy.Fuzzy_Out))*myVehicle.speed_auto;
	}
	PID_Setpoint(&pid_MR, MPS2RPS(v_long_right));
	PID_Setpoint(&pid_ML, MPS2RPS(v_long_left));
}

void	Vehicle_TestFuzzy(void) {
	double yaw_current;
	double v_long_left, v_long_right;

	yaw_current	= myVehicle.position_center_veh.yaw;

	// Fuzzy Controller
	myFuzzy.Set_Angle 	= Pi_To_Pi(myVehicle.target_yaw);
	myFuzzy.Angle		= yaw_current;
	Fuzzy_UpdateInput(&myFuzzy);
	myFuzzy.Fuzzy_Out = Fuzzy_Defuzzification_Max_Min(myFuzzy.Fuzzy_Error, myFuzzy.Fuzzy_Error_dot);
	// Update setpoint
	if (myFuzzy.Fuzzy_Out >= 0) {
		// Turn left
		v_long_right 	= ( fabs(myFuzzy.Fuzzy_Out))*myVehicle.speed_auto;// (meter/second)
		v_long_left		= (- fabs(myFuzzy.Fuzzy_Out))*myVehicle.speed_auto;
	} else {
		// Turn right
		v_long_right 	= (- fabs(myFuzzy.Fuzzy_Out))*myVehicle.speed_auto;
		v_long_left		= ( fabs(myFuzzy.Fuzzy_Out))*myVehicle.speed_auto;
	}
	PID_Setpoint(&pid_MR, MPS2RPS(v_long_right));
	PID_Setpoint(&pid_ML, MPS2RPS(v_long_left));

	// Check stable
	double angle_error;
	angle_error = Pi_To_Pi(myVehicle.target_yaw - yaw_current);
	if (fabs(angle_error) < PI/36) {
		myVehicle.time_out_rotate_stable += 0.05;
	} else {
		myVehicle.time_out_rotate_stable = 0;
	}
}

void	Vehicle_AutoNewTarget(double target_x, double target_y, uint8_t *target_data) {
	// Test fuzzy
	myVehicle.target_yaw = target_x;

	// Save target
	//	myVehicle.target_x = target_x; // Target of Lidar, need to convert to center
	myVehicle.target_y = target_y;
	memcpy(&(myVehicle.target_frame[0]), target_data, LENGHT_CMD_AUTO_MOVE);
	// Switch state
	myVehicle.state_auto_run = AUTO_RUN_NEW_TARGET;
	Vehicle_Stop();
}

void	Vehicle_AutoStart() {
	myVehicle.state_auto_run = AUTO_RUN_MOVING;
}

void	Vehicle_AutoStop() {
	Vehicle_Stop();
	myVehicle.state_auto_run = AUTO_RUN_WAIT_START;
}

void	Vehicle_AutoRunState() {
	switch (myVehicle.state_auto_run) {
		case AUTO_RUN_IDLE:
			// Do nothing
			__NOP();
			break;
		case AUTO_RUN_NEW_TARGET:
			__NOP();
//			// Start point
//			myVehicle.start_x = myVehicle.position_lidar.x;
//			myVehicle.start_y = myVehicle.position_lidar.y;
//			// Init new path in stanley controller
//			if (Stanley_InitNewPath(&myStanley,
//								myVehicle.target_x, myVehicle.target_y,
//								myVehicle.start_x, myVehicle.start_y,
//								myVehicle.position_center_veh.yaw)) {
//				// Go to the next state
//				myVehicle.state_auto_run = AUTO_RUN_WAIT_START;
//			} else {
//				// Return wait new target
//				myVehicle.state_auto_run = AUTO_RUN_IDLE;
//			}
			myVehicle.state_auto_run = AUTO_RUN_WAIT_START;
			break;
		case AUTO_RUN_WAIT_START:
			// Do nothing
			__NOP();
			break;
		case AUTO_RUN_MOVING:
			//Vehicle_AutoDrive();
			Vehicle_TestFuzzy();
			if (myVehicle.time_out_rotate_stable >= 1) {
				myVehicle.state_auto_run = AUTO_RUN_FINISH;
			}

			// Check finish
//			if (Stanley_IsFinish(&myStanley)) {
//				Vehicle_Stop();
//				myVehicle.state_auto_run = AUTO_RUN_FINISH;
//			}
			break;
		case AUTO_RUN_FINISH:
			// Stop vehicle
			Vehicle_Stop();
			// Inform Computer
			AtSerial_ReportFinishTarget(myVehicle.target_frame);
			// Return wait new target
			myVehicle.state_auto_run = AUTO_RUN_IDLE;
			break;
		case AUTO_RUN_ERROR:
			break;
		default:
			break;
	}
}


void Vehicle_Stop(void) {
	myVehicle.move_manual = STOP_VEHICLE;
	myVehicle.time_out_move_manual = 0;
	PID_Setpoint(&pid_MR, 0);
	PID_Setpoint(&pid_ML, 0);
}

void Vehicle_Forward(void) {
	myVehicle.move_manual = FORWARD;
	myVehicle.time_out_move_manual = 0;
	PID_Setpoint(&pid_MR, 1.5);
	PID_Setpoint(&pid_ML, 1.5);
}

void Vehicle_Backward(void) {
	myVehicle.move_manual = BACKWARD;
	myVehicle.time_out_move_manual = 0;
	PID_Setpoint(&pid_MR, -0.8);
	PID_Setpoint(&pid_ML, -0.8);
}

void Vehicle_RotLeft(void) {
	myVehicle.move_manual = ROTATE_LEFT;
	myVehicle.time_out_move_manual = 0;
	PID_Setpoint(&pid_MR, 0.4);
	PID_Setpoint(&pid_ML, -0.4);
}

void Vehicle_RotRight(void) {
	myVehicle.move_manual = ROTATE_RIGHT;
	myVehicle.time_out_move_manual = 0;
	PID_Setpoint(&pid_MR, -0.4);
	PID_Setpoint(&pid_ML, 0.4);
}

void Vehicle_ForwardLeft(void) {
	myVehicle.move_manual = FORWARD_LEFT;
	myVehicle.time_out_move_manual = 0;
	PID_Setpoint(&pid_MR, 1.5);
	PID_Setpoint(&pid_ML, 1);
}

void Vehicle_ForwardRight(void) {
	myVehicle.move_manual = FORWARD_RIGHT;
	myVehicle.time_out_move_manual = 0;
	PID_Setpoint(&pid_MR, 1);
	PID_Setpoint(&pid_ML, 1.5);
}

void Vehicle_BackwardLeft(void) {
	myVehicle.move_manual = BACKWARD_LEFT;
	myVehicle.time_out_move_manual = 0;
	PID_Setpoint(&pid_MR, -1);
	PID_Setpoint(&pid_ML, -0.6);
}

void Vehicle_BackwardRight(void) {
	myVehicle.move_manual = BACKWARD_RIGHT;
	myVehicle.time_out_move_manual = 0;
	PID_Setpoint(&pid_MR, -0.6);
	PID_Setpoint(&pid_ML, -1);
}


void Vehicle_CheckManualTimeOut(void) {
	myVehicle.time_out_move_manual += 0.01;
	if (myVehicle.time_out_move_manual > TIME_LIMIT_MOVE_REFRESH) {
		Vehicle_Stop();
	}
}



