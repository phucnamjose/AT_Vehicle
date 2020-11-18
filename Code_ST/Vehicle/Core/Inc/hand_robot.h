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

typedef enum enum_StateScanHard{
	SCAN_HARD_UP = 0,
	SCAN_HARD_DOWN,
	SCAN_HARD_DONE
}enum_StateHard;

typedef enum enum_StateScan{
	SCAN_STEP_INIT = 0,
	SCAN_STEP_HARD,
	SCAN_STEP_SOFT,
	SCAN_STEP_FINISH
}enum_StateScan;

/* Object form*/
typedef struct Hand_Duty_t
{
	Trajectory_t	trajec[2];
	double			angle_start[2];
	double			time_total;
}Hand_Duty_t;

typedef struct Hand_t
{
	// Scan work
	enum_StateScan 	state_scan;
	enum_StateHard	state_hard;
	uint8_t			is_scanned;
	// Position
	double			offset_up;
	double			offset_down;
	double			theta_up;
	double			theta_down;
	// Limit
	double			lim_neg_up;
	double			lim_neg_down;
	double			lim_pos_down;
	// Duty
	Hand_Duty_t		duty;
	//
}Hand_t;


/* Function prototypes */
void	Hand_Init(void);
uint8_t	Hand_Scan(void);
uint8_t	Hand_ScanHard(void);
uint8_t Hand_GoToSoftLim(void);
uint8_t	Hand_IsScanned(void);

void	Hand_GoHome(void);

void 	Hand_Stop(void);
void 	Hand_Up(void);
void 	Hand_Down(void);
void 	Hand_Left(void);
void 	Hand_Right(void);

void	Hand_UpdateTruePositon(void);
void	Hand_ExcuteNewPosition(void);

#endif /* INC_HAND_ROBOT_H_ */
