/*
 * geometry.c
 *
 *  Created on: Nov 25, 2020
 *      Author: Dang Nam
 */

#include "geometry.h"
#include "math.h"

/* Compute line parameters by normal vector and 1 point that line go via */
void lineInit(Line_t *line, Point_t point_via, Point_t point_orient) {
	double normal_x, normal_y;

	normal_x = point_orient.x - point_via.x;
	normal_y = point_orient.y - point_via.y;

	line->a = normal_x;
	line->b = normal_y;
	line->c = - normal_x*point_via.x - normal_y*point_via.y;
}

double lineDis2Point(Line_t line, Point_t point_consider, Point_t point_compare) {
	double dis_num, dis_den;
	double distance;
	double point1_in_line, point2_in_line;
	double sign_check;

	dis_num = fabs(line.a*point_consider.x + line.b*point_consider.y + line.c);
	dis_den = sqrt(line.a*line.a + line.b*line.b);

	point1_in_line = line.a*point_consider.x + line.b*point_consider.y + line.c;
	point2_in_line = line.a*point_compare.x + line.b*point_compare.y + line.c;

	sign_check = point1_in_line*point2_in_line;

	if (sign_check > 0) {
		distance = dis_num / dis_den; // same side
	} else if (sign_check < 0) {
		distance = - dis_num / dis_den; // dif side
	} else {
		distance = 0; // in line
	}

	return distance;
}


