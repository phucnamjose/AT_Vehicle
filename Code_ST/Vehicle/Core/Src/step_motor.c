/*
 * step_motor.c
 *
 *  Created on: Nov 7, 2020
 *      Author: Dang Nam
 */


#include "step_motor.h"


int32_t pulse_accumulate[4];

double  offset_setpoint[4];
int32_t offset_encoder[3];
int32_t offset_stepper;

uint8_t	limit_switch[4];
uint8_t state_scan;
uint8_t scan_flag;

const int8_t	pulse_scan[4] = {3, 5, 5, 10};
