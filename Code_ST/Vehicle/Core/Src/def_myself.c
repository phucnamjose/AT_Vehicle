/*
 * def_myself.c
 *
 *  Created on: Oct 26, 2020
 *      Author: Dang Nam
 */

#include "def_myself.h"
#include "system_params.h"

int32_t	double2string( uint8_t *result, double value, uint8_t precision) {
	uint8_t nguyen[4];
	uint8_t le[6];
	int8_t sign;
	double temp1, temp2;
	int32_t index;

	if((precision < 0) || (6 < precision)) {
		precision = 6;
	}

	if(value < 0) {
		sign = -1;
	} else {
		sign = 1;
	}
	value = value*sign;

	if (value > 10000.0f) {
		return -1;
	}

	nguyen[0] 	= (int32_t)value/1000;
	nguyen[1]	= (int32_t)value/100 - nguyen[0]*10;
	nguyen[2]	= (int32_t)value/10 - nguyen[0]*100 - nguyen[1]*10;
	nguyen[3]	= (int32_t)value - nguyen[0]*1000 - nguyen[1]*100 - nguyen[2]*10;

	temp1	= (value - nguyen[0]*1000 - nguyen[1]*100 - nguyen[2]*10 - nguyen[3])*1000;
	le[0] 	= (int32_t)temp1/100;
	le[1] 	= (int32_t)temp1/10 - le[0]*10;
	le[2]	= (int32_t)temp1 - le[0]*100 - le[1]*10;

	temp2	= (temp1 - le[0]*100 - le[1]*10 - le[2])*1000;
	le[3] 	= (int32_t)temp2/100;
	le[4] 	= (int32_t)temp2/10 - le[3]*10;
	le[5]	= (int32_t)temp2 - le[3]*100 - le[4]*10;
	// Rounding
	if ((temp2 - le[3]*1000 - le[4]*100 - le[5]) >= 0.5) {
		le[5]++;
	}

	index = 0;

	if( -1 == sign) {
		*(result + index++) = '-'; // Negative
	}
	// Find first position
	if ( value < 1) {
		*(result + index++) = 0x30;
	} else {
		int8_t i = 0;
		for( ; i < 4; i++) {
			if (nguyen[i] > 0) {
				*(result + index++) = nguyen[i] + 0x30;
				i++;
				break;
			}
		}

		for( ; i < 4; i++) {
		*(result + index++) = nguyen[i] + 0x30;
		}
	}
	*(result + index++) = '.';
	for( int8_t i = 0; i < precision; i++) {
			*(result + index++) = le[i] + 0x30;
	}

	// Chua giai quyet van de lam tron chu so thap phan
	*(result + index++) = 0;
	return index;
}

double filter(double alpha, double x, double pre_x) {
	return (1 - alpha)*x + alpha*pre_x;
}

/** @brief  : Convert m/s to round per second
**  @agr    : Input velocity in m/s from wheel
**  @retval : RPS value
**/
double MPS2RPS(double vel)
{
	return ((vel / (2.0 * PI * WHEEL_RADIUS * RATIO_MOTOR_WHEEL)));
}

/** @brief  : Convert round per second to m/s
**  @agr    : Input angular velocity from motor
**  @retval : MPS value
**/
double	RPS2MPS(double vel) {
	return (vel * 2.0 * PI * WHEEL_RADIUS * RATIO_MOTOR_WHEEL);
}

/** @brief  : Pi to Pi
**  @agr    : input angle
**  @retval : double value
**/
double Pi_To_Pi(double angle)
{
	if(angle > PI)
		angle = angle - 2.0 * PI;
	else if (angle < -PI)
		angle = angle + 2.0 * PI;

	return angle;
}
