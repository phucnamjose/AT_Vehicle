/*
 * def_myself.h
 *
 *  Created on: Oct 25, 2020
 *      Author: Dang Nam
 */

#ifndef INC_DEF_MYSELF_H_
#define INC_DEF_MYSELF_H_

#include "stm32f4xx_hal.h"

// Boolean value
#define FALSE	(0)
#define TRUE	(1)

// Pi constant
#define PI		(3.14159265359f)

// Null pointer
#define NULL	((void *)0)

int32_t	double2string( uint8_t *result, double value, uint8_t precision);
double 	filter(double alpha, double x, double pre_x);
double	MPS2RPS(double vel);
double	RPS2MPS(double vel);
double 	Pi_To_Pi(double angle);

#endif /* INC_DEF_MYSELF_H_ */
