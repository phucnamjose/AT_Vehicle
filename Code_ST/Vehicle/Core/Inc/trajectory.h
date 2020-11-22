/*
 * trajectory.h
 *
 *  Created on: Nov 11, 2020
 *      Author: Dang Nam
 */

#ifndef INC_TRAJECTORY_H_
#define INC_TRAJECTORY_H_

#include "def_myself.h"

typedef enum
{
	  TRAJEC_INIT_QVA		= 0x00U,  /*!< Consume A max, determine T max */
	  TRAJEC_INIT_QVT		= 0x01U  /*!< Consume T max, determine A max  */
}ModeInit_t;

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

uint8_t	trajecInitLSPB	(Trajectory_t *lspb,
						double total_s,
						ModeInit_t modeinit,
						double v_design,
						double a_design);
uint8_t	trajecFlowLSPB	(Trajectory_t *lspb, double time);

#endif /* INC_TRAJECTORY_H_ */
