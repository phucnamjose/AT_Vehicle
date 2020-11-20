/*
 * hand_robot.h
 *
 *  Created on: Oct 31, 2020
 *      Author: Dang Nam
 */

#ifndef INC_HAND_ROBOT_H_
#define INC_HAND_ROBOT_H_

#include "def_myself.h"
#include "trajectory.h"
#include "debug_cmd.h"

#define MAX_SPEED_STEP_UP		(0.1) // rad/s
#define MAX_SPEED_STEP_DOWN		(0.49087)

#define TIME_LIMIT_HAND_REFRESH	(1.5) // second

typedef enum enum_HandStateAuto{
	HAND_AUTO_IDLE = 0,
	HAND_AUTO_INIT,
	HAND_AUTO_RUNNING,
	HAND_AUTO_FINISH
}enum_HandStateAuto;

typedef enum enum_HandScanHard{
	SCAN_HARD_UP = 0,
	SCAN_HARD_DOWN,
	SCAN_HARD_DONE
}enum_HandScanHard;

typedef enum enum_StateScan{
	SCAN_STEP_INIT = 0,
	SCAN_STEP_HARD,
	SCAN_STEP_SOFT,
	SCAN_STEP_FINISH
}enum_HandStateScan;

typedef enum enum_StepAction{
	STEP_STOP = 0,
	STEP_GO_POSITIVE,
	STEP_GO_NEGATIVE
}enum_StepAction;


/* Object form*/
typedef struct Hand_Duty_t
{
	Trajectory_t	trajec[2];
	double			angle_start[2];
	double			time_total;
}Hand_Duty_t;

typedef struct Hand_t
{
	// Common
	enum_HandMode		mode;
	double				speed;// percent
	uint8_t				flag_error;
	// Manual
	enum_HandManual		type_manual;
	double				timeout_hand_manual;
	// Auto
	enum_HandAuto		type_auto;
	enum_HandStateAuto	state_auto;
	// Scan work
	enum_HandStateScan 	state_scan;
	enum_HandScanHard	state_hard;
	uint8_t				is_scanned;
	// Position
	double				offset_up;
	double				offset_down;
	double				theta_up;
	double				theta_down;
	// Limit
	double				lim_neg_up;
	double				lim_neg_down;
	double				lim_pos_down;
	// Duty
	Hand_Duty_t			duty;
	// Joint
	double				speed_up_current;
	double				speed_down_current;
	enum_StepAction		action_up;
	enum_StepAction		action_down;
	enum_StepAction		action_up_pre;
	enum_StepAction		action_down_pre;
}Hand_t;


/* Function prototypes */
// Common
void	Hand_Init(void);
void	Hand_ChangeMode(enum_HandMode mode);
void	Hand_ChangeSpeed(double percent);
// Lowlevel
uint8_t	Hand_Scan(void);
uint8_t	Hand_ScanHard(void);
uint8_t Hand_GoToSoftLim(void);
uint8_t	Hand_IsScanned(void);
void	Hand_UpdateTruePositon(void);
void	Hand_ExcuteNewPosition(void);
// Auto
void	Hand_AutoRunState(void);
// Manual
void 	Hand_Stop(void);
void 	Hand_Up(void);
void 	Hand_Down(void);
void 	Hand_Left(void);
void 	Hand_Right(void);
void	Hand_GoHome(void);
void	Hand_ManualRun(void);

#endif /* INC_HAND_ROBOT_H_ */
