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
#include "pid_heading.h"
#include "FIFO_buff.h"


/* External Variables */
extern DcServo_t 	MR;
extern DcServo_t 	ML;
extern PID_t		pid_MR;
extern PID_t		pid_ML;
extern Fuzzy_t     	myFuzzy;
extern HeadPID_t	head_pid;
extern Stanley_t  	myStanley;

extern FifoBuff_t fifo_delta_x;
extern FifoBuff_t fifo_delta_y;
extern FifoBuff_t fifo_delta_yaw;
/* Internal Variables */
Vehicle_t 		myVehicle;


void	Vehicle_Init() {
	myVehicle.mode_vehicle 	= MODE_MANUAL;
	myVehicle.move_manual	= STOP_VEHICLE;
	myVehicle.state_auto_run = AUTO_RUN_IDLE;
	myVehicle.time_vel_manual = 0;
	myVehicle.phase_manual	= PHASE_MAN_DEC;
	myVehicle.time_out_move_manual 	= 0;
	myVehicle.miss_lidar_count		= 0;
	myVehicle.speed_manual			= 1;
	myVehicle.speed_auto			= 1; // 100%
	myVehicle.count_lidar			= 0;
	myVehicle.count_fusion			= 0;
	myVehicle.count_auto			= 0;
	Fuzzy_Init();
	Head_PID_Init();
	Stanley_Init(&myStanley);
}


void Vehicle_ChangeMode(enum_ModeVehicle mode) {
	switch (mode) {
		case MODE_MANUAL:
			myVehicle.mode_vehicle = mode;
			Vehicle_StopHard();
			break;
		case MODE_AUTO:
			myVehicle.mode_vehicle = mode;
			Vehicle_StopHard();
			break;
		default:
			break;
	}
}

void	Vehicle_ChangeSpeed(double speed) {
	if (speed >= 0 && speed <= 1) {
	//	myVehicle.speed_manual = speed;
	//	myVehicle.speed_auto = speed;
	}
}


void	Vehicle_Localization(void) {
	// Run odometry per 10ms
	Vehicle_Odometry();
	myVehicle.count_lidar++;
	// Get Lidar per 1s
	if (myVehicle.count_lidar == 1) {
		AtSerial_RequestPosition();
	} else if (myVehicle.count_lidar == 100) {
		// Reset count_lidar
		myVehicle.count_lidar = 0;
	}
	// Check new position frame from Lidar and read
	uint32_t position_count = AtSerial_GetPositionCount();
	if (position_count != myVehicle.position_lidar_count) {
		AtSerial_GetPosition(&(myVehicle.position_lidar));
		myVehicle.position_lidar_count = position_count;
		Vehicle_EstimatePosition(TRUE);
	} else {
		Vehicle_EstimatePosition(FALSE);
	}
}

void Vehicle_EstimatePosition(uint8_t has_lidar) {
	double x_center, y_center, heading;

	if (has_lidar) {
		// Note: Yaw(Lidar) != Yaw(Vehicle -- heading angle)
		heading = Pi_To_Pi(myVehicle.position_lidar.yaw - HEADING_OFFSET_LIDAR);
		// Center is in the middle 2 wheel
		x_center = myVehicle.position_lidar.x + DIS_LIDAR_FRONT_WHEEL*cos(heading);
		y_center = myVehicle.position_lidar.y + DIS_LIDAR_FRONT_WHEEL*sin(heading);
		// Update center
		myVehicle.position_center_veh.x 	= x_center;
		myVehicle.position_center_veh.y 	= y_center;
		myVehicle.position_center_veh.yaw 	= heading;

		// Add odometry delta because of delay
		myVehicle.count_fusion++;
		if (myVehicle.count_fusion == 1) {
			myVehicle.count_fusion = 0;
			double delta_x, delta_y, r, delta_theta, x_fusion, y_fusion, yaw_fusion;
			delta_x = FifoGetSum(&fifo_delta_x);
			delta_y = FifoGetSum(&fifo_delta_y);
			r = sqrt(delta_x*delta_x + delta_y*delta_y);
			delta_theta = FifoGetSum(&fifo_delta_yaw);
			yaw_fusion = Pi_To_Pi(heading + delta_theta);
			x_fusion = x_center + r*cos(yaw_fusion);
			y_fusion = y_center + r*sin(yaw_fusion);
			// Update fusion
			myVehicle.position_fusion.x 	= x_fusion;
			myVehicle.position_fusion.y 	= y_fusion;
			myVehicle.position_fusion.yaw 	= yaw_fusion;
			// Update base position for odometry method
			memcpy(&(myVehicle.position_odometry),
				&(myVehicle.position_fusion), sizeof(Position_t));
		}
		myVehicle.miss_lidar_count 	= 0;
		myVehicle.miss_lidar_flag	= FALSE;
	} else {
		// Use result from odometry
		memcpy(&(myVehicle.position_fusion),
			&(myVehicle.position_odometry), sizeof(Position_t));
		// Check miss lidar. If > 10s == Stop
		if (++myVehicle.miss_lidar_count > 10) {
			myVehicle.miss_lidar_flag	= TRUE;
		}
	}
}

void	Vehicle_Odometry() {
	double delta_left, delta_right, delta_yaw;
	double x_new, y_new, yaw_new;
	double yaw_old;
	double delta_xb, delta_yb;
	double delta_x, delta_y;
	double v_bx;

	yaw_old = myVehicle.position_odometry.yaw;
	// vb
	delta_left	= RPS2MPS(DcGetVel(&ML))*BASIC_PERIOD*FIX_EXPER;
	delta_right	= RPS2MPS(DcGetVel(&MR))*BASIC_PERIOD*FIX_EXPER;
	v_bx		= 0.5*(delta_left + delta_right);
	delta_yaw 	= (delta_right - delta_left)/(DIS_TWO_WHEELS);// DIS = 2L in formula
	// delta q_b
	if (fabs(delta_yaw) < 0.00001) {
		delta_xb = v_bx;
		delta_yb = 0;
	} else {
		delta_xb = v_bx*sin(delta_yaw)/delta_yaw;
		delta_yb = v_bx*(1 - cos(delta_yaw))/delta_yaw;
	}
	// convert to base frame
	delta_x = delta_xb*cos(yaw_old) - delta_yb*sin(yaw_old);
	delta_y = delta_xb*sin(yaw_old) + delta_yb*cos(yaw_old);

	x_new 	= myVehicle.position_odometry.x + delta_x;
	y_new	= myVehicle.position_odometry.y + delta_y;
	yaw_new	= Pi_To_Pi(yaw_old + delta_yaw);// [-Pi; Pi]
	// Update
	myVehicle.position_odometry.x 	= x_new;
	myVehicle.position_odometry.y 	= y_new;
	myVehicle.position_odometry.yaw = yaw_new;
	// Save delta to FIFO buffer
	FifoPushSample(&fifo_delta_x, delta_x);
	FifoPushSample(&fifo_delta_y, delta_y);
	FifoPushSample(&fifo_delta_yaw, delta_yaw);
}

void Vehicle_ResetOdometry(void) {
	memset(&(myVehicle.position_odometry), 0, sizeof(Position_t));
}

void	Vehicle_CopyLidar(void) {
	memcpy(&(myVehicle.position_odometry),
			&(myVehicle.position_center_veh), sizeof(Position_t));
}

void Vehicle_AutoDrive(void) {
	double v_left, v_right;
	double v_vehicle;
	double x_current, y_current, yaw_current;
	double v_long_left, v_long_right;
	double delta_angle;
	// current pose
	x_current 	= myVehicle.position_fusion.x;
	y_current	= myVehicle.position_fusion.y;
	yaw_current	= myVehicle.position_fusion.yaw;
	// current linear velocity
	v_left		= RPS2MPS(DcGetVel(&ML));
	v_right		= RPS2MPS(DcGetVel(&MR));
	v_vehicle	= 0.5*(v_left + v_right);
	// Stanley Controller
	Stanley_Follow(&myStanley, x_current, y_current, yaw_current, v_vehicle);
	// Fuzzy Controller
	delta_angle = myStanley.Delta_Angle;
	myFuzzy.Set_Angle 	= Pi_To_Pi(yaw_current + delta_angle);
	myFuzzy.Angle		= yaw_current;
	Fuzzy_UpdateInput(&myFuzzy);
	myFuzzy.Fuzzy_Out = Fuzzy_Defuzzification_Max_Min(myFuzzy.Fuzzy_Error,
													myFuzzy.Fuzzy_Error_dot);
	// Change auto linear speed
	double speed;
	// Accelerate
	if (myStanley.distance_begin_line < 0)
	{
		speed = myVehicle.speed_auto*VEH_MIN_AUTO;
	} else if (myStanley.distance_begin_line < 0.4) {
		speed = myVehicle.speed_auto*(VEH_MIN_AUTO
				+ (VEH_MAX_AUTO-VEH_MIN_AUTO)*myStanley.distance_begin_line/0.4);
	} else {
		speed = myVehicle.speed_auto*VEH_MAX_AUTO;
	}
	// Decelerate
	if (myStanley.distance_goal_line < 0.4) {
		speed = myVehicle.speed_auto*(VEH_MAX_AUTO
				- (VEH_MAX_AUTO-VEH_MIN_AUTO)*(0.4 - myStanley.distance_goal_line)/0.4);
	} else if (myStanley.distance_goal_line < 0) {
		speed = myVehicle.speed_auto*VEH_MIN_AUTO;
	}

	// Update setpoint
	myVehicle.speed_auto_current = speed;
	if (myFuzzy.Fuzzy_Out >= 0) {
		// Turn left
		v_long_right = (1 + fabs(myFuzzy.Fuzzy_Out))*myVehicle.speed_auto_current;
		v_long_left	 = (1 - fabs(myFuzzy.Fuzzy_Out))*myVehicle.speed_auto_current;
	} else {
		// Turn right
		v_long_right = (1 - fabs(myFuzzy.Fuzzy_Out))*myVehicle.speed_auto_current;
		v_long_left	 = (1 + fabs(myFuzzy.Fuzzy_Out))*myVehicle.speed_auto_current;
	}

	// Check if cross-track error ->0 , thetae -> 0 then go straigh, not through fuzzy
	if ((fabs(myStanley.efa) < 0.02) && (fabs(myStanley.Thetae) < 0.04)) {
		v_long_right 	= myVehicle.speed_auto_current;
		v_long_left		= myVehicle.speed_auto_current;
	}

	Vehicle_SetLinearVel(v_long_right, v_long_left);// m/s
}

void	Vehicle_RotateFuzzy(void) {
	double yaw_current;
	double v_long_left, v_long_right;

	yaw_current	= myVehicle.position_fusion.yaw;

	// Fuzzy Controller
	myFuzzy.Set_Angle 	= Pi_To_Pi(myVehicle.target_yaw);
	myFuzzy.Angle		= yaw_current;
	Fuzzy_UpdateInput(&myFuzzy);
	myFuzzy.Fuzzy_Out = Fuzzy_Defuzzification_Max_Min(myFuzzy.Fuzzy_Error,
													myFuzzy.Fuzzy_Error_dot);
	// Update setpoint
	myVehicle.speed_auto_current = myVehicle.speed_auto*VEH_AUTO_ROTATE;
	if (myFuzzy.Fuzzy_Out >= 0) {
		// Turn left
		v_long_right 	= (fabs(myFuzzy.Fuzzy_Out))*myVehicle.speed_auto_current;
		v_long_left		= (-fabs(myFuzzy.Fuzzy_Out))*myVehicle.speed_auto_current;
	} else {
		// Turn right
		v_long_right 	= (-fabs(myFuzzy.Fuzzy_Out))*myVehicle.speed_auto_current;
		v_long_left		= (fabs(myFuzzy.Fuzzy_Out))*myVehicle.speed_auto_current;
	}
	Vehicle_SetLinearVel(v_long_right, v_long_left);// m/s

	// Check stable
	double angle_error;
	angle_error = Pi_To_Pi(myVehicle.target_yaw - yaw_current);
	if (fabs(angle_error) < PI/72) {
		myVehicle.time_out_rotate_stable += 0.05;
	} else {
		myVehicle.time_out_rotate_stable = 0;
	}
}

void	Vehicle_TestHeadPID(void) {
	double yaw_current;
	double v_long_left, v_long_right;
	double output;

	yaw_current	= myVehicle.position_fusion.yaw;
	// PID heading controller
	myVehicle.speed_auto_current = myVehicle.speed_auto*VEH_AUTO_ROTATE;
	Head_PID_SetSaturation(&head_pid, 2*myVehicle.speed_auto_current);
	Head_PID_SetPoint(&head_pid, Pi_To_Pi(myVehicle.target_yaw));
	Head_PID_UpdateFeedback(&head_pid, yaw_current);
	Head_PID_Compute(&head_pid);
	output = Head_PID_GetOutputEach(&head_pid);

	// Update setpoint
	v_long_right 	= -output; // m/s
	v_long_left		= output;
	Vehicle_SetLinearVel(v_long_right, v_long_left);

	// Check stable
	double angle_error;
	angle_error = Pi_To_Pi(myVehicle.target_yaw - yaw_current);
	if (fabs(angle_error) < PI/72) {
		myVehicle.time_out_rotate_stable += 0.05;
	} else {
		myVehicle.time_out_rotate_stable = 0;
	}
}

void	Vehicle_AutoNewTarget(double target_x, double target_y, uint8_t *target_data) {
//	// Test fuzzy
//	myVehicle.target_yaw = target_x;

	// Save target
	 myVehicle.target_x = target_x;
	myVehicle.target_y = target_y;
	memcpy(&(myVehicle.target_frame[0]), target_data, LENGHT_CMD_AUTO_MOVE);
	// Switch state
	myVehicle.state_auto_run = AUTO_RUN_NEW_TARGET;
	Vehicle_StopHard();
}

void	Vehicle_AutoStart() {
	myVehicle.state_auto_run = AUTO_RUN_MOVING;
}

void	Vehicle_AutoStop() {
	Vehicle_StopHard();
	myVehicle.state_auto_run = AUTO_RUN_WAIT_START;
}

void	Vehicle_AutoRunState() {
	switch (myVehicle.state_auto_run) {
		case AUTO_RUN_IDLE:
			// Do nothing
			break;
		case AUTO_RUN_NEW_TARGET:
			// Start point
			myVehicle.start_x = myVehicle.position_fusion.x;
			myVehicle.start_y = myVehicle.position_fusion.y;
			// Init new path in stanley controller
			if (Stanley_InitNewPath(&myStanley,
								myVehicle.target_x, myVehicle.target_y,
								myVehicle.start_x, myVehicle.start_y,
								myVehicle.position_fusion.yaw)) {
				// Go to the next state
				myVehicle.state_auto_run = AUTO_RUN_WAIT_START;
			} else {
				// Return wait new target
				myVehicle.state_auto_run = AUTO_RUN_FINISH;
			}
			break;
		case AUTO_RUN_WAIT_START:
			myVehicle.count_auto = 0;
			break;
		case AUTO_RUN_MOVING:
			myVehicle.count_auto++;
			if (myVehicle.count_auto == 5) {
				myVehicle.count_auto = 0; // Period 50ms
				// Rotate before if it's need
				if (Stanley_CheckRotateFlag()) {
					myVehicle.target_yaw = myStanley.angle_go;
					Vehicle_RotateFuzzy();
					if (fabs(myFuzzy.Fuzzy_Error) < PI/8)
						Stanley_ClearRotateFlag();
				} else {
					// Moving linear
					Vehicle_AutoDrive();
					 //Check finish
					if (Stanley_IsFinish(&myStanley)) {
						myVehicle.state_auto_run = AUTO_RUN_FINISH;
						Vehicle_StopHard();
					}
				}
			}
			break;
		case AUTO_RUN_FINISH:
			// Stop vehicle
			Vehicle_StopHard();
			// Return wait new target
			myVehicle.state_auto_run = AUTO_RUN_IDLE;
			// Inform Computer
			AtSerial_ReportFinishTarget(myVehicle.target_frame);
			break;
		case AUTO_RUN_ERROR:
			break;
		default:
			break;
	}
}


void	Vehicle_SetLinearVel(double v_long_right, double v_long_left) {
	PID_Setpoint(&pid_MR, MPS2RPS(v_long_right));
	PID_Setpoint(&pid_ML, MPS2RPS(v_long_left));
}

void Vehicle_StopSoft(void) {
	myVehicle.pre_move_manual = myVehicle.move_manual;
	myVehicle.move_manual = STOP_VEHICLE;
	myVehicle.time_out_move_manual = 0;
}

void Vehicle_StopHard(void) {
	myVehicle.move_manual = STOP_VEHICLE;
	myVehicle.pre_move_manual = STOP_VEHICLE;
	myVehicle.time_out_move_manual = 0;
	Vehicle_SetLinearVel(0, 0);
	myVehicle.speed_man_current = 0;
}

void Vehicle_Forward(void) {
	myVehicle.move_manual = FORWARD;
	myVehicle.time_out_move_manual = 0;
}

void Vehicle_Backward(void) {
	myVehicle.move_manual = BACKWARD;
	myVehicle.time_out_move_manual = 0;
}

void Vehicle_RotLeft(void) {
	myVehicle.move_manual = ROTATE_LEFT;
	myVehicle.time_out_move_manual = 0;
	Vehicle_SetLinearVel(0.07, -0.07);
}

void Vehicle_RotRight(void) {
	myVehicle.move_manual = ROTATE_RIGHT;
	myVehicle.time_out_move_manual = 0;
	Vehicle_SetLinearVel(-0.07, 0.07);
}

void Vehicle_ForwardLeft(void) {
	myVehicle.move_manual = FORWARD_LEFT;
	myVehicle.time_out_move_manual = 0;
	Vehicle_SetLinearVel(0.489, 0.326);
}

void Vehicle_ForwardRight(void) {
	myVehicle.move_manual = FORWARD_RIGHT;
	myVehicle.time_out_move_manual = 0;
	Vehicle_SetLinearVel(0.326, 0.489);
}

void Vehicle_BackwardLeft(void) {
	myVehicle.move_manual = BACKWARD_LEFT;
	myVehicle.time_out_move_manual = 0;
	Vehicle_SetLinearVel(-0.195, -0.326);
}

void Vehicle_BackwardRight(void) {
	myVehicle.move_manual = BACKWARD_RIGHT;
	myVehicle.time_out_move_manual = 0;
	Vehicle_SetLinearVel(-0.326, -0.195);
}


void Vehicle_ManualRun(void) {
	switch (myVehicle.move_manual) {
		case STOP_VEHICLE:
			if (FORWARD == myVehicle.pre_move_manual) {
				double vel_set, max_speed;
				max_speed = myVehicle.speed_manual*VEH_MAX_MANUAL_FORWARD;
				vel_set = myVehicle.speed_man_current - 0.01*max_speed;
				vel_set = (vel_set < 0) ? 0 : vel_set;
				myVehicle.speed_man_current = vel_set;
				Vehicle_SetLinearVel(vel_set, vel_set);
			} else if (BACKWARD == myVehicle.pre_move_manual) {
				double vel_set, max_speed;
				max_speed = myVehicle.speed_manual*VEH_MAX_MANUAL_BACKWARD;
				vel_set = myVehicle.speed_man_current + 0.01*max_speed;
				vel_set = (vel_set > 0) ? 0 : vel_set;
				myVehicle.speed_man_current = vel_set;
				Vehicle_SetLinearVel(vel_set, vel_set);
			} else {
				Vehicle_SetLinearVel(0, 0);
				myVehicle.speed_man_current = 0;
			}
			break;
		case FORWARD:
			{
				double vel_set, max_speed;
				max_speed = myVehicle.speed_manual*VEH_MAX_MANUAL_FORWARD;
				vel_set = myVehicle.speed_man_current + 0.01*max_speed;
				vel_set = (vel_set > max_speed) ? max_speed : vel_set;
				myVehicle.speed_man_current = vel_set;
				Vehicle_SetLinearVel(vel_set, vel_set);
			}
			break;
		case BACKWARD:
			{
				double vel_set, max_speed;
				max_speed = myVehicle.speed_manual*VEH_MAX_MANUAL_BACKWARD;
				vel_set = myVehicle.speed_man_current - 0.01*max_speed;
				vel_set = (vel_set < (-max_speed)) ? -max_speed : vel_set;
				myVehicle.speed_man_current = vel_set;
				Vehicle_SetLinearVel(vel_set, vel_set);
			}
			break;
		default:
			break;
	}
	// Check timeout
	if (myVehicle.move_manual > STOP_VEHICLE) {
		myVehicle.time_out_move_manual += 0.01;
		if (myVehicle.time_out_move_manual > TIME_LIMIT_MOVE_REFRESH) {
			Vehicle_StopSoft();
		}
	}
}



