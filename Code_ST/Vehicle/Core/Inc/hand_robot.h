/*
 * hand_robot.h
 *
 *  Created on: Oct 31, 2020
 *      Author: Dang Nam
 */

#ifndef INC_HAND_ROBOT_H_
#define INC_HAND_ROBOT_H_

#include "def_myself.h"


typedef struct Trajectory_t
{
	int8_t			 dir;
	double			 s0;
	double 			 s1;
	double			 v0;
	double			 v1;
	double 			 v_design;
	double 			 a_design;
	double			 v_lim;
	double			 Ta;
	double			 Td;
	double			 Tf;
	uint32_t		 num_of_sampling;
	double			 total_s;
	double			 a_current;
	double			 v_current;
	double			 s_current;

}Trajectory_t;


typedef struct Hand_Duty_t
{
	Trajectory_t	trajec[2];
	double			angle_start[2];
	double			time_total;
}Hand_Duty_t;



void 	Hand_ScanJointUp(void);
void 	Hand_ScanJointDown(void);
void 	Hand_Stop(void);
void 	Hand_Up(void);
void 	Hand_Down(void);
void 	Hand_Left(void);
void 	Hand_Right(void);

void	Hand_UpdateTruePositon(void);
void	Hand_ExcuteNewPosition(void);

uint8_t	Hand_InitLSPB(Trajectory_t *lspb,
					double total_s,
					double v_factor,
					double a_factor);
uint8_t	Hand_FlowLSPB(Trajectory_t *lspb, double time);

#endif /* INC_HAND_ROBOT_H_ */
