/*
 * geometry.h
 *
 *  Created on: Nov 25, 2020
 *      Author: Dang Nam
 */

#ifndef INC_GEOMETRY_H_
#define INC_GEOMETRY_H_

#include "def_myself.h"


typedef struct Point_t
{
	double x, y;
}Point_t;

typedef struct Line_t
{
	double a, b, c; // ax + by + c = 0
}Line_t;


/* Public fuction prototype */
void	 lineInit(Line_t *line, Point_t via_point, Point_t orient_point);
double	 lineDis2Point(Line_t line, Point_t point_consider, Point_t point_compare);

#endif /* INC_GEOMETRY_H_ */
