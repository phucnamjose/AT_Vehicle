/*
 * trajectory.h
 *
 *  Created on: Nov 11, 2020
 *      Author: Dang Nam
 */

#ifndef INC_TRAJECTORY_H_
#define INC_TRAJECTORY_H_

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

#endif /* INC_TRAJECTORY_H_ */
