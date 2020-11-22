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

#define MAX_SPEED_STEP_UP		(0.096) // rad/s
#define MAX_SPEED_STEP_DOWN		(0.49087)

#define ANGLE_HOME_UP			(-1.62315204) // rad
#define ANGLE_HOME_DOWN			(0.0)

#define TIME_LIMIT_HAND_REFRESH	(1.5) // second

typedef enum enum_HandStateAuto {
	HAND_AUTO_INIT = 0,
	HAND_AUTO_RUNNING,
	HAND_AUTO_FINISH
}enum_HandStateAuto;

typedef enum enum_HandScanHard {
	SCAN_HARD_UP = 0,
	SCAN_HARD_DOWN,
	SCAN_HARD_DONE
}enum_HandScanHard;

typedef enum enum_StateScan {
	SCAN_STEP_INIT = 0,
	SCAN_STEP_HARD,
	SCAN_STEP_FINISH
}enum_HandStateScan;

typedef enum enum_StepAction {
	STEP_STOP = 0,
	STEP_GO_POSITIVE,
	STEP_GO_NEGATIVE
}enum_StepAction;

typedef enum enum_DutyState {
	DUTY_STATE_READY = 0,
	DUTY_STATE_INIT,
	DUTY_STATE_FOLLOW,
	DUTY_STATE_FINISH
}enum_DutyState;

typedef enum enum_HomeState {
	HOME_STATE_INIT = 0,
	HOME_STATE_FOLLOW,
	HOME_STATE_FINISH
}enum_HomeState;

/* Object form*/
typedef struct Hand_Duty_t
{
	Trajectory_t	trajec[2];
	double			angle_start[2];
	double			time_total;
	double			time_run;
	uint8_t			is_finished;
	enum_DutyState	state_duty;
}Hand_Duty_t;

typedef struct Hand_t
{
	// Common
	enum_HandMode		mode;
	enum_HandMode		mode_next_after_home;
	double				speed;// percent
	uint8_t				flag_error;
	// None - Go home mode
	enum_HomeState		state_homing;
	// Manual mode
	enum_HandManual		type_manual;
	double				timeout_hand_manual;
	// Auto mode
	enum_HandAuto		type_auto;
	enum_HandStateAuto	state_auto;
	uint8_t				flag_home_soft;
	uint8_t				index_rectangle;
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
	double				speed_bound_up_pos;
	double				speed_bound_up_neg;
	double				speed_bound_down_pos;
	double				speed_bound_down_neg;
}Hand_t;


/* Function prototypes */
// Common
void	Hand_Init(void);
void	Hand_ChangeMode(enum_HandMode mode);
void	Hand_ChangeSpeed(double percent);
void	Hand_Run(void);
// Lowlevel
uint8_t	Hand_Scan(void);
uint8_t	Hand_ScanHard(void);
uint8_t Hand_GoToSoftLim(void);
uint8_t	Hand_IsScanned(void);
void	Hand_UpdateTruePositon(void);
uint8_t	Hand_ExcuteNewPosition(double theta_down, double theta_up);
// Auto
void	Hand_AutoRun(void);
uint8_t	Hand_InitDuty(Hand_Duty_t *duty, double goal_down, double goal_up);
uint8_t Hand_FollowDuty(Hand_Duty_t *duty);
uint8_t	Hand_IsFinishedDuty(Hand_Duty_t *duty);
// Manual
void	Hand_UpdateSpeedBound(void);
void 	Hand_Stop(void);
void 	Hand_Up(void);
void 	Hand_Down(void);
void 	Hand_Left(void);
void 	Hand_Right(void);
void	Hand_ManualRun(void);
// None mode
void	Hand_StartGoHome(void);
void	Hand_HomeRun(void);

#endif /* INC_HAND_ROBOT_H_ */
