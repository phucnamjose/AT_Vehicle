/*
 * main_thread.c
 *
 *  Created on: Oct 25, 2020
 *      Author: Dang Nam
 */
#include "dc_servo.h"
#include "pid_motor.h"
#include "def_myself.h"
#include "main_thread.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "tx_dma_manage.h"
#include "arduino.h"
#include "vehicle_overall.h"
#include "main.h"
#include "pid_heading.h"
#include "step_motor.h"
#include "hand_robot.h"

/* External variable*/
// Peripherals MCU
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim9;
// User
extern DcServo_t 		MR;
extern DcServo_t 		ML;
extern PID_t 			pid_MR;
extern PID_t 			pid_ML;
extern HeadPID_t	head_pid;
extern Vehicle_t 		myVehicle;
extern Step_t			stepDown;
extern Step_t			stepUp;
extern Hand_t			myHand;
extern osMailQId 		mainTaskMailHandle;
extern ValueUsing_t 	valueUsingTable;

/* Internal variable*/
// Counter
uint8_t 		count_report;
uint8_t			count_auto_run;
uint8_t			count_read_head;
// Buffer
uint8_t 		usb_out_buff[200];
uint8_t			pi_out_buff[200];
uint8_t 		setR[20];
uint8_t 		setL[20];
uint8_t			fbackR[20];
uint8_t			fbackL[20];
uint8_t 		kp_buff[20];
uint8_t 		ki_buff[20];
uint8_t 		kd_buff[20];
uint8_t 		v_buff[20];
uint8_t 		yaw_buff[20];
// Mail
mainTaskMail_t 	command;
mainTaskMail_t 	*mail;
osEvent 		is_new_mail;
uint8_t			is_new_command;
// State
uint8_t			Execute_flag;
uint8_t			Run_PID_flag;
uint8_t			Report_flag;
uint8_t			Read_heading_flag;

/* Implementation*/

/* Run one shot when mainTask begin */
void setupMainThread(void) {
	// Pull-up Resistor to PC Detect
	HAL_GPIO_WritePin(USB_PULL_GPIO_Port, USB_PULL_Pin, GPIO_PIN_SET);
	// Start timer for Servo
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	// Init DcServo motor and PID controller
	DcServoInit(&MR, 1, 1, TIM2, TIM3);
	DcServoInit(&ML, 0, 0, TIM1, TIM4);
	PID_Init();
	// Stop motor
	DcStopMotor(&MR);
	DcStopMotor(&ML);
	// Init vehicle
	Vehicle_Init();
	// Init Stepper motor and Hand
	HAL_TIM_PWM_Stop_IT(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop_IT(&htim9, TIM_CHANNEL_1);
	StepInit(&stepUp, 0, 3, TIM5);
	StepInit(&stepDown, 0, 3, TIM9);
	Hand_Init();
	// Turn power
	//AtSerial_SetPowerMotion(1);
	// Init state
	Run_PID_flag 		= TRUE;
	Execute_flag 		= TRUE;
	Report_flag			= FALSE;
	Read_heading_flag	= FALSE;
	// Start timer for basic period
	HAL_TIM_Base_Start_IT(&htim7);
}

/* Repeat per basic period */
void loopMainThread(void) {
	// 1.Clear usb buffer
	memset(usb_out_buff, 0, sizeof(usb_out_buff));
	// 2.Update velocity
	DcVelUpdate(&MR);
	DcVelUpdate(&ML);

	// 3.Localization
	Vehicle_Localization();

	// 4.Run According to Mode Vehicle
	switch (myVehicle.mode_vehicle) {
		case MODE_MANUAL:
			Vehicle_RunManual();
			break;

		case MODE_AUTO:
//			if (Head_IsRun()) {
//				Head_RunBlackBox();
//				Head_PushSample(myVehicle.position_center_veh.yaw);
//			}
			// Run test fuzzy first
			Vehicle_AutoRunState();
			break;
		default:
			break;
	}

	// 5.Run PID
	if (Run_PID_flag) {
		//PID_RunBlackBox(&pid_ML, &ML);
		PID_Compute(&pid_MR, &MR);
		PID_Compute(&pid_ML, &ML);
	}
	// 6.Execute Output
	if (Execute_flag) {
		DcExecuteOuput(&MR);
		DcExecuteOuput(&ML);
	}

	// 7. Hand action

	// 8.Check mail
	is_new_mail = osMailGet(mainTaskMailHandle, 0);// If have no mail, skip
	if (is_new_mail.status == osEventMail) {
		// Copy mail
		mail = is_new_mail.value.p;
		memcpy(&command, mail, sizeof(mainTaskMail_t));
		osMailFree(mainTaskMailHandle, mail);
		is_new_command = TRUE;
	}
	// 9.Check new command
	if (is_new_command) {
		decisionAccordingCmd(command);
		// Reset
		is_new_command = FALSE;
	}

//	// Report heading sample
//	if (Read_heading_flag) {
//		double v_output, head_respond;
//		v_output = Head_PopOutput(count_read_head);
//		head_respond = Head_PopSample(count_read_head);
//		count_read_head++;
//
//		double2string(v_buff, v_output, 6);
//		double2string(yaw_buff, head_respond, 6);
//		// Send through USB
//		char feedback[30];
//		snprintf(feedback, 30, "%d %s %s\n",(int)count_read_head, v_buff, yaw_buff);
//		strcat((char *)usb_out_buff, feedback);
//
//		if (count_read_head == 200)
//			Read_heading_flag = FALSE;
//	}

	// 10.Report per 100ms
	if (Report_flag) {
		count_report++;
		if (count_report == 10) {
			// Reset count report
			count_report = 0;

//			//System identify BEGIN -----------------------------------------
//			double out_left;
//			double out_right;
//			int32_t pid_count;
//			// Get setpoint and feedback
//			pid_count 	= PID_GetCount(&pid_ML);
//			out_left		= PID_GetOutput(&pid_ML);
//			out_right		= PID_GetOutput(&pid_MR);
//			double2string(setR, out_right, 6);
//			double2string(setL, out_left, 6);
//			char fb_and_output[60];
//			int32_t len_pi = snprintf(fb_and_output, 60, "%d %s %s\r\n",
//								 (int)pid_count, fbackL, setL);
//			if (pid_count == 0)
//				Report_flag = FALSE;
//			//System identify END ------------------------------------------


//			//Report Servo respond BEGIN -----------------------------------------
//			double v_lelf;
//			double v_right;
//			double sp_left;
//			double sp_right;
//			v_right		= DcGetVel(&MR);
//			v_lelf	 	= DcGetVel(&ML);
//			sp_right	= PID_GetSetpoint(&pid_MR);
//			sp_left		= PID_GetSetpoint(&pid_ML);
//			// Convert to string
//			double2string(fbackR, v_right, 6);
//			double2string(fbackL, v_lelf, 6);
//			double2string(setR, sp_right, 6);
//			double2string(setL, sp_left, 6);
//			// Send through USB
//			char feedback[30];
//			snprintf(feedback, 30, "%s %s\n", fbackR, fbackL);
//			strcat((char *)usb_out_buff, feedback);
//			// Send through UART
//			char fb_and_sp[60];
//			int32_t len_pi = snprintf(fb_and_sp, 60, "%s %s %s %s\r\n",
//					fbackR, fbackL, setR, setL);
//			serial_sendRasberryPi((uint8_t *)fb_and_sp, len_pi);
//			//Report Servo respond END ------------------------------------------


//			//Report Limit Switch BEGIN ------------------------------------------
//			uint8_t ls_up, ls_down_left, ls_down_right;
//			StepReadLimit(&stepUp, &stepDown);
//			ls_up = stepUp.limit_negative;
//			ls_down_left = stepDown.limit_negative;
//			ls_down_right = stepDown.limit_positive;
//			// Send through USB
//			char feedback[30];
//			snprintf(feedback, 30, "%d %d %d\n", (int)ls_up, (int)ls_down_left, (int)ls_down_right);
//			strcat((char *)usb_out_buff, feedback);
//			//Report Limit Switch END ------------------------------------------

			double angle, x, y;
			angle = myVehicle.position_center_veh.yaw;
			x	= myVehicle.position_center_veh.x;
			y	= myVehicle.position_center_veh.y;
			char angle_buff[20];
			char x_buff[20];
			char y_buff[20];
			char report_angle[60];
			double2string((uint8_t *)angle_buff, angle, 6);
			double2string((uint8_t *)x_buff, x, 6);
			double2string((uint8_t *)y_buff, y, 6);
			snprintf(report_angle, 60, "%s %s %s\n", angle_buff, x_buff, y_buff);
			strcat((char *)usb_out_buff, report_angle);
		}
	}
	// 10. Check usb buff and send
	int32_t lenght;
	lenght = strlen((char *)usb_out_buff);
	if (lenght > 0)
		CDC_Transmit_FS(usb_out_buff, lenght);

}

void decisionAccordingCmd(mainTaskMail_t cmd) {
	switch (cmd.cmd_code) {
		// 1.
		case SET_SETPOINT:
			PID_Setpoint(&pid_MR, cmd.setpoint_right);
			PID_Setpoint(&pid_ML, cmd.setpoint_left);
			strcat((char *)usb_out_buff, "Changed SETPOINT\n");
			break;
		// 2.
		case SET_PID:
			PID_SetFactor(&pid_MR, cmd.R_kp, cmd.R_ki, cmd.R_kd);
			PID_SetFactor(&pid_ML, cmd.L_kp, cmd.L_ki, cmd.L_kd);
			strcat((char *)usb_out_buff, "Changed PID\n");
			break;
		// 3.
		case START:
			Run_PID_flag 	= TRUE;
			Execute_flag	= TRUE;
			strcat((char *)usb_out_buff, "Started EXE\n");
			break;
		// 4.
		case STOP:
			DcStopMotor(&MR);
			DcStopMotor(&ML);
			Run_PID_flag 	= FALSE;
			Execute_flag	= FALSE;
			strcat((char *)usb_out_buff, "Stopped EXE\n");
			break;
		// 5.
		case SAVE_PID:
			// Write Flash file
			strcat((char *)usb_out_buff, "Saved PID to Flash\n");
			break;
		// 6.
		case GET_PID:
		{
			double kp_r, ki_r, kd_r;
			double kp_l, ki_l, kd_l;
			// Read factor
			PID_GetFactor(&pid_MR, &kp_r, &ki_r, &kd_r);
			PID_GetFactor(&pid_ML, &kp_l, &ki_l, &kd_l);
			// Convert right
			double2string(kp_buff, kp_r, 6);
			double2string(ki_buff, ki_r, 6);
			double2string(kd_buff, kd_r, 6);
			char report[50];
			snprintf(report, 50, "Right: %s %s %s\n", kp_buff, ki_buff, kd_buff);
			strcat((char *)usb_out_buff, report);
			// Convert left
			double2string(kp_buff, kp_l, 6);
			double2string(ki_buff, ki_l, 6);
			double2string(kd_buff, kd_l, 6);
			snprintf(report, 50, "Left: %s %s %s\n", kp_buff, ki_buff, kd_buff);
			strcat((char *)usb_out_buff, report);
		}
			break;
		// 7.
		case RESET_PID:
			PID_Reset(&pid_MR);
			PID_Reset(&pid_ML);
			strcat((char *)usb_out_buff, "RESET PID\n");
			break;
		// 8.
		case MOVE_MANUAL:
			moveManualVehicle(cmd.move);
			break;
		// 9.
		case REPORT_ON:
			Report_flag = TRUE;
			break;
		// 10.
		case REPORT_OFF:
			Report_flag = FALSE;
			break;
		// 11.
		case SET_OUTPUT:
			DcSetOuput(&MR, cmd.R_output);
			DcSetOuput(&ML, cmd.L_output);
			strcat((char *)usb_out_buff, "Changed OUTPUT\n");
			break;
		// 12.
		case HAND_MANUAL:
			// TODO:
			break;
		// 13.
		case GET_SAMPLE:
			Read_heading_flag = TRUE;
			break;
		// 14.
		case RESET_ODOMETRY:
			Vehicle_ResetOdometry();
			strcat((char *)usb_out_buff, "Reset ODOMETRY\n");
			break;
		// 15.
		case COPY_LIDAR_2_ODO:

			break;
		// 16.
		case MODE_VEHICLE:
			Vehicle_ChangeMode(cmd.mode_vehicle);
			strcat((char *)usb_out_buff, "Changed MODE\n");
			break;
		// 17.
		case MOVE_AUTO:
			moveAutoVehicle(cmd.target_x, cmd.target_y, cmd.target_frame);
			break;
		// 18.
		case SPEED_MOVE:
			Vehicle_ChangeSpeed(cmd.speed_move);
			strcat((char *)usb_out_buff, "Changed SPEED\n");
			break;
		// 19.
		case EMERGENCY:
			// TODO:
			break;
		// 20.
		case HAND_AUTO:
			// TODO:
			break;
		// 21.
		case AUTO_START:
			autoStart();
			break;
		// 22.
		case AUTO_STOP:
			autoStop();
			break;
		// 23.
		case AUTO_ROTATE:
			// TODO:
			break;
		default:
			break;
	}
}

uint8_t moveManualVehicle(enum_MoveManual move_type) {

	// Check mode
	if (MODE_AUTO == myVehicle.mode_vehicle)
		return FALSE;
	// Right mode
	switch (move_type) {
		case STOP_VEHICLE:
			Vehicle_StopSoft();
			break;
		case FORWARD:
			Vehicle_Forward();
			break;
		case BACKWARD:
			Vehicle_Backward();
			break;
		case ROTATE_LEFT:
			Vehicle_RotLeft();
			break;
		case ROTATE_RIGHT:
			Vehicle_RotRight();
			break;
		case FORWARD_LEFT:
			Vehicle_ForwardLeft();
			break;
		case FORWARD_RIGHT:
			Vehicle_ForwardRight();
			break;
		case BACKWARD_LEFT:
			Vehicle_BackwardLeft();
			break;
		case BACKWARD_RIGHT:
			Vehicle_BackwardRight();
			break;
		default:
			break;
	}
	return TRUE;
}

uint8_t moveAutoVehicle(double x, double y, uint8_t *target_data) {
	// Check mode
	if (MODE_MANUAL == myVehicle.mode_vehicle)
		return FALSE;

	if (AUTO_RUN_IDLE == myVehicle.state_auto_run
			|| AUTO_RUN_WAIT_START == myVehicle.state_auto_run) {
		Vehicle_AutoNewTarget(x, y, target_data);
		return TRUE;
	} else
		return FALSE;
}

uint8_t	autoStart(void) {
	// Check mode
	if (MODE_MANUAL == myVehicle.mode_vehicle)
		return FALSE;

	if (AUTO_RUN_WAIT_START == myVehicle.state_auto_run) {
		Vehicle_AutoStart();
		return TRUE;
	} else
		return FALSE;
}

uint8_t	autoStop(void) {
	// Check mode
	if (MODE_MANUAL == myVehicle.mode_vehicle)
		return FALSE;

	if (AUTO_RUN_MOVING == myVehicle.state_auto_run) {
		Vehicle_AutoStop();
		return TRUE;
	} else
		return FALSE;
}

void 	handManualRobot(enum_HandManual hand_manual) {

}

void 	handAutoRobot(enum_HandManual hand_manual) {

}

void mainTask_SendMail(mainTaskMail_t *cmd_to_main) {
	mainTaskMail_t *mainMail;
	mainMail = NULL;

	// Allocate mail memory
	while (mainMail == NULL) {
		mainMail = osMailAlloc(mainTaskMailHandle, osWaitForever);
	}
	// Copy to mail
	memcpy(mainMail, cmd_to_main, sizeof(mainTaskMail_t));
	// Send mail
	osMailPut(mainTaskMailHandle, mainMail);
}
