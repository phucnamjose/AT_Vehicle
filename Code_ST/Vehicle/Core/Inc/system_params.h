/*
 * system_params.h
 *
 *  Created on: Oct 25, 2020
 *      Author: Dang Nam
 */

#ifndef INC_SYSTEM_PARAMS_H_
#define INC_SYSTEM_PARAMS_H_

#include "def_myself.h"

//	Process
#define BASIC_PERIOD						(0.01)
//	Vehicle sructure
#define WHEEL_RADIUS	 					(0.075)
#define DIS_TWO_WHEELS  				   	(0.3225)
#define DIS_LIDAR_FRONT_WHEEL				(0.37351)
#define	HEADING_OFFSET_LIDAR				(0)
#define RATIO_MOTOR_WHEEL					(9.0/13.0)
//	Vehicle limit
#define SPEED_MAX_VEHICLE					(0.6) // (m/s)
//	Hand structure
#define RATIO_STEP_UP						(1/98)
#define RATIO_STEP_DOWN						(1/5)
//	Hand limit

#endif /* INC_SYSTEM_PARAMS_H_ */
