/*
 * pid_heading.h
 *
 *  Created on: Nov 11, 2020
 *      Author: Dang Nam
 */

#ifndef INC_PID_HEADING_H_
#define INC_PID_HEADING_H_

#include "def_myself.h"

void	Head_StartBlackBox(void);
uint8_t	Head_RunBlackBox(void);
void	Head_SetVel(double vel);
void	Head_PushSample(double val);
double	Head_PopSample(int32_t index);
double	Head_PopOutput(int32_t index);
uint8_t	Head_IsRun(void);

#endif /* INC_PID_HEADING_H_ */
