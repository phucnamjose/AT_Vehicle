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
#define WHEEL_RADIUS	 					(0.074961978)
#define DIS_TWO_WHEELS  				   	(0.3225)
#define DIS_LIDAR_FRONT_WHEEL				(0.37351)
#define	HEADING_OFFSET_LIDAR				(0.082612933)
#define RATIO_MOTOR_WHEEL					(9.0/13.0)
#define FIX_EXPER							(1.0/1.033333333)
//	Hand structure
#define RESOL_STEP_UP						(200)
#define RESOL_STEP_DOWN						(400)

#define MICRO_STEP_UP						(16)
#define MICRO_STEP_DOWN						(16)

#define RATIO_STEP_UP						(1.0/98.0)
#define RATIO_STEP_DOWN						(1.0/5.0)

#define LEVEL_ACTIVE_LS_UP_POS				(1)
#define LEVEL_ACTIVE_LS_DOWN_NEG			(0)
#define LEVEL_ACTIVE_LS_DOWN_POS			(0)
//	Hand limit
#define LIM_PULSE_UP						50
#define LIM_PULSE_DOWN						50

#define	HARD_LIM_UP_NEG						(-2.094395102) // -2pi/3
#define	HARD_LIM_UP_POS						(-1.570796327) // -pi/2

#define	HARD_LIM_DOWN_NEG					(-1.39820508) // -80 deg
#define	HARD_LIM_DOWN_POS					(1.39820508)  // 80 deg

#define	SOFT_LIM_UP_NEG						(-2.00712864) // range 34 deg
#define	SOFT_LIM_UP_POS						(-1.62315204)

#define	SOFT_LIM_DOWN_NEG					(-1.343903524) // range 170 deg
#define	SOFT_LIM_DOWN_POS					(1.343903524)

#endif /* INC_SYSTEM_PARAMS_H_ */
