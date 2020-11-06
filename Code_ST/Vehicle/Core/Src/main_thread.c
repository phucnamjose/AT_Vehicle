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
//#include "MPU9250.h"

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
extern float roll,yaw,pitch;
extern float q0, q1, q2, q3;
extern DcServo_t MR;
extern DcServo_t ML;
extern PID_t pid_MR;
extern PID_t pid_ML;
extern osMailQId mainTaskMailHandle;
extern ValueUsing_t valueUsingTable;

/* Internal variable*/
// USB
uint8_t 		count;
uint8_t			count_imu;
uint8_t 		usb_out_buff[200];
uint8_t			pi_out_buff[200];
uint8_t 		setR[20];
uint8_t 		setL[20];
uint8_t			fbackR[20];
uint8_t			fbackL[20];
uint8_t 		kp_buff[20];
uint8_t 		ki_buff[20];
uint8_t 		kd_buff[20];
uint8_t 		roll_buff[20];
uint8_t 		pitch_buff[20];
uint8_t 		yaw_buff[20];
uint8_t			quar_buff[20];

// Mail
mainTaskMail_t 	command;
mainTaskMail_t 	*mail;
osEvent 		is_new_mail;
uint8_t			is_new_command;
// State
uint8_t			Execute_flag;
uint8_t			Run_PID_flag;
uint8_t			Send_PID_flag;
uint8_t			Report_flag;

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
	DcServoInit(&MR, 1, 0, TIM2, TIM3);
	DcServoInit(&ML, 0, 1, TIM1, TIM4);
	PID_Reset(&pid_MR);
	PID_Reset(&pid_ML);
	PID_SetFactor(&pid_MR,
			PID_DEFAULT_RIGHT_KP,
			PID_DEFAULT_RIGHT_KI,
			PID_DEFAULT_RIGHT_KD);
	PID_SetFactor(&pid_ML,
			PID_DEFAULT_LEFT_KP,
			PID_DEFAULT_LEFT_KI,
			PID_DEFAULT_LEFT_KD);
	// Stop motor
	DcStopMotor(&MR);
	DcStopMotor(&ML);

	//Init IMU
//	init_IMU();
//	init_magnetometer();
//	if (Check_Connection(0x71) == TRUE) {
//		Calibration_IMU();
//		HAL_GPIO_WritePin(LED_D4_GPIO_Port, LED_D4_Pin, GPIO_PIN_RESET);
//		osDelay(1000);
//		HAL_GPIO_WritePin(LED_D4_GPIO_Port, LED_D4_Pin, GPIO_PIN_SET);
//	} else {
//		uint8_t err[40] = "IMU Init FAIL..........\n";
//		CDC_Transmit_FS(err, strlen((const char *)err));
//	}
	// Init state
	Run_PID_flag 	= FALSE;
	Execute_flag 	= FALSE;
	Send_PID_flag 	= FALSE;
	// Start timer for basic period
	HAL_TIM_Base_Start_IT(&htim7);
}

/* Repeat per basic period */
void loopMainThread(void) {
	// 1.Clear usb buffer
	memset(usb_out_buff, 0, sizeof(usb_out_buff));
	memset(usb_out_buff, 0, sizeof(pi_out_buff));
	// 2.Read IMU
//	Process_IMU();
//	count_imu++;
//	if (count_imu == 10) {
//		count_imu = 0;
//		double2string(roll_buff, roll, 6);
//		double2string(pitch_buff, pitch, 6);
//		double2string(yaw_buff, yaw, 6);
//		//double2string(quar_buff, q3, 6);
//		char orientation[50];
//		snprintf(orientation, 50, "%s %s %s\n", roll_buff, pitch_buff, yaw_buff);
//		strcat((char *)usb_out_buff, orientation);
//	}

	// 3.Update velocity
	DcVelUpdate(&MR);
	DcVelUpdate(&ML);

	// 4.Run PID
	if (Run_PID_flag) {
		PID_RunBlackBox(&pid_ML, &ML);
		//PID_Compute(&pid_MR, &MR);
		//PID_Compute(&pid_ML, &ML);
	}

	// 5.Execute Output
	if (Execute_flag) {
		//DcExecuteOuput(&MR);
		DcExecuteOuput(&ML);
	}

	// 6.Check mail
	is_new_mail = osMailGet(mainTaskMailHandle, 0);// If have no mail, skip
	if (is_new_mail.status == osEventMail) {
		// Copy mail
		mail = is_new_mail.value.p;
		memcpy(&command, mail, sizeof(mainTaskMail_t));
		osMailFree(mainTaskMailHandle, mail);
		is_new_command = TRUE;
	}

	// 7.Check new command
	if (is_new_command) {
		decisionAccordingCmd(command);
		// Reset
		is_new_command = FALSE;
	}

	// 8.Report per 10ms
	if (Report_flag) {
		count++;
		if (count == 1) {
			double v_lelf, v_right;
			double sp_left, sp_right;
			double out_left, out_right;
			// Get setpoint and feedback
			//v_right		= DcGetVel(&MR);
			v_lelf	 	= DcGetVel(&ML);
//			sp_right	= PID_GetSetpoint(&pid_MR);
//			sp_left	= PID_GetSetpoint(&pid_ML);

			//sp_right	= PID_GetOutput(&pid_MR);
			out_left		= PID_GetOutput(&pid_MR);


			// Convert to string
//			double2string(fbackR, v_right, 6);
			double2string(fbackL, v_lelf, 6);
//			double2string(setR, sp_right, 6);
//			double2string(setL, sp_left, 6);
//			double2string(setR, sp_right, 6);
			double2string(setL, out_left, 6);

//			char feedback[30];
//			snprintf(feedback, 30, "%s %s\n", fbackR, fbackL);
//			strcat((char *)usb_out_buff, feedback);

//			char fb_and_sp[60];
//			int32_t len_pi = snprintf(fb_and_sp, 60, "%s %s %s %s\r\n",
//					fbackR, fbackL, setR, setL);

			char fb_and_output[60];
			int32_t len_pi = snprintf(fb_and_output, 60, "%s %s\r\n",
								fbackL, setL);


			serial_sendRasberryPi((uint8_t *)fb_and_output, len_pi);
			// Reset count
			count = 0;
		}
	}
	// 9. Check usb buff and send
	int32_t lenght;
	lenght = strlen((char *)usb_out_buff);
	if (lenght > 0)
		CDC_Transmit_FS(usb_out_buff, lenght);

}

void decisionAccordingCmd(mainTaskMail_t cmd) {
	switch (cmd.cmd_code) {
		case SET_SETPOINT:
			PID_Setpoint(&pid_MR, cmd.setpoint_right);
			PID_Setpoint(&pid_ML, cmd.setpoint_left);
			strcat((char *)usb_out_buff, "Changed SETPOINT\n");
			break;
		case SET_PID:
			PID_SetFactor(&pid_MR, cmd.R_kp, cmd.R_ki, cmd.R_kd);
			PID_SetFactor(&pid_ML, cmd.L_kp, cmd.L_ki, cmd.L_kd);
			strcat((char *)usb_out_buff, "Changed PID\n");
			break;
		case START:
			Run_PID_flag 	= TRUE;
			Execute_flag	= TRUE;
			PID_StartBlackBox(&pid_ML);
			strcat((char *)usb_out_buff, "Started EXE\n");
			break;
		case STOP:
			DcStopMotor(&MR);
			DcStopMotor(&ML);
			Run_PID_flag 	= FALSE;
			Execute_flag	= FALSE;
			strcat((char *)usb_out_buff, "Stopped EXE\n");
			break;
		case SAVE_PID:
			// Write Flash file
			strcat((char *)usb_out_buff, "Saved PID to Flash\n");
			break;
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
		case RESET_PID:
			PID_Reset(&pid_MR);
			PID_Reset(&pid_ML);
			strcat((char *)usb_out_buff, "RESET PID\n");
			break;
		case MOVE:
			// Move file
			moveVehicle(cmd.move);
			break;
		case REPORT_ON:
			Report_flag = TRUE;
			break;
		case REPORT_OFF:
			Report_flag = FALSE;
			break;
		case SET_OUTPUT:
			DcSetOuput(&MR, cmd.R_output);
			DcSetOuput(&ML, cmd.L_output);
			strcat((char *)usb_out_buff, "Changed OUTPUT\n");
			break;
		case POWER_ON:
			AtSerial_SetPowerMotion(1);
			strcat((char *)usb_out_buff, "Turned on POWER\n");
			break;
		case POWER_OFF:
			AtSerial_SetPowerMotion(0);
			strcat((char *)usb_out_buff, "Turned off POWER\n");
			break;
		default:
			break;
	}
}

void moveVehicle(enum_Move move_type) {
	switch (move_type) {
		case STOP_VEHICLE:
			DcStopMotor(&MR);
			DcStopMotor(&ML);
			Vehicle_Stop();
			Run_PID_flag 	= FALSE;
			Execute_flag	= FALSE;
			Vehicle_Stop();
			strcat((char *)usb_out_buff, "STOP\n");
			break;
		case FORWARD:
			Vehicle_Forward();
			Run_PID_flag 	= TRUE;
			Execute_flag	= TRUE;
			strcat((char *)usb_out_buff, "FORWARD\n");
			break;
		case BACKWARD:
			Vehicle_Backward();
			Run_PID_flag 	= TRUE;
			Execute_flag	= TRUE;
			strcat((char *)usb_out_buff, "BACKWARD\n");
			break;
		case ROTATE_LEFT:
			Vehicle_RotLeft();
			Run_PID_flag 	= TRUE;
			Execute_flag	= TRUE;
			strcat((char *)usb_out_buff, "ROTATE LEFT\n");
			break;
		case ROTATE_RIGHT:
			Vehicle_RotRight();
			Run_PID_flag 	= TRUE;
			Execute_flag	= TRUE;
			strcat((char *)usb_out_buff, "ROTATE RIGHT\n");
			break;
		case FORWARD_LEFT:
			Vehicle_ForwardLeft();
			Run_PID_flag 	= TRUE;
			Execute_flag	= TRUE;
			strcat((char *)usb_out_buff, "FORWARD-LEFT\n");
			break;
		case FORWARD_RIGHT:
			Vehicle_ForwardRight();
			Run_PID_flag 	= TRUE;
			Execute_flag	= TRUE;
			strcat((char *)usb_out_buff, "FORWARD-RIGHT\n");
			break;
		case BACKWARD_LEFT:
			Vehicle_BackwardLeft();
			Run_PID_flag 	= TRUE;
			Execute_flag	= TRUE;
			strcat((char *)usb_out_buff, "BACKWARD-LEFT\n");
			break;
		case BACKWARD_RIGHT:
			Vehicle_BackwardRight();
			Run_PID_flag 	= TRUE;
			Execute_flag	= TRUE;
			strcat((char *)usb_out_buff, "BACKWARD-RIGHT\n");
			break;
		default:
			break;
	}
}

void handRobot(enum_Hand hand_type) {

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
