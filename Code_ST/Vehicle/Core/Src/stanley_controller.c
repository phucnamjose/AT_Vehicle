/*
 * stanley_controller.c
 *
 *  Created on: Nov 2, 2020
 *      Author: Dang Nam
 */

#include "stanley_controller.h"
#include "flash_rdwr.h"
#include "system_params.h"
#include "fuzzy_controller.h"
#include "math.h"


/* Internal variables */
Stanley_t  myStanley;

/*
** @brief  : Initial value for Stanley functions
**  @agr    : input
**  @retval : Return fix value
**/
void Stanley_Init(Stanley_t *stanley) {
	stanley->Goal_Flag = FALSE;
	stanley->refPointIndex = -1; // there is no point that was referred
	stanley->K = 1.2;
	stanley->Ksoft = 0.15;
	stanley->Step = 0.2;
}

uint8_t		Stanley_InitNewPath(Stanley_t *stanley,
								double target_x, double target_y,
								double start_x, double start_y,
								double yaw) {
	double	x_start_real, y_start_real;
	double	x_end_real, y_end_real;
	double	delta_x, delta_y;
	//double	angle_fake;
	double	angle_real;
	double	lenght_fake;
	double	lenght_real;

	delta_x 	= target_x - start_x;
	delta_y 	= target_y - start_y;
	lenght_fake = sqrt(delta_x*delta_x + delta_y*delta_y);
	// Check lenght 2 point
	if (lenght_fake < 0.01)
		return FALSE;

//	x_start_real = start_x + DIS_LIDAR_FRONT_WHEEL*cos(yaw);
//	y_start_real = start_y + DIS_LIDAR_FRONT_WHEEL*sin(yaw);

	x_start_real = start_x;
	y_start_real = start_y;

	//angle_fake = atan2(delta_y, delta_x);
	x_end_real = target_x;
	y_end_real = target_y;

	//x_end_real = target_x + DIS_LIDAR_FRONT_WHEEL*cos(angle_fake);
	//y_end_real = target_y + DIS_LIDAR_FRONT_WHEEL*sin(angle_fake);

	delta_x 	= x_end_real - x_start_real;
	delta_y 	= y_end_real - y_start_real;
	angle_real	= atan2(delta_y, delta_x);
	lenght_real = sqrt(delta_x*delta_x + delta_y*delta_y);

	int32_t	number_point;
	// Star point and middle point
	number_point = lenght_real / STEP_REFER;
	for (int32_t i = 0; i < number_point; i++) {
		stanley->P_X[i] = x_start_real + STEP_REFER * i * cos(angle_real);
		stanley->P_Y[i] = y_start_real + STEP_REFER * i * sin(angle_real);
	}
	// End point
	stanley->P_X[number_point] = x_end_real;
	stanley->P_Y[number_point] = y_end_real;
	stanley->NbOfP = number_point + 1;
	stanley->NbOfWayPoints = stanley->NbOfP;
	// Heading path
	Stanley_UpdateRefYaw(stanley);
	// Start line and goal line

	myStanley.point_begin.x = x_start_real;
	myStanley.point_begin.y = y_start_real;
	myStanley.point_goal.x 	= x_end_real;
	myStanley.point_goal.y	= y_end_real;
	lineInit(&(myStanley.line_begin), myStanley.point_begin, myStanley.point_goal);
	lineInit(&(myStanley.line_goal), myStanley.point_goal, myStanley.point_begin);
	// Check to rotate
	myStanley.angle_go = angle_real;
	if (fabs(Pi_To_Pi(angle_real - yaw)) > PI/6) {
		myStanley.flag_rotate_first = TRUE;
	} else {
		myStanley.flag_rotate_first = FALSE;
	}
	// Reset
	stanley->refPointIndex = -1;
	stanley->Goal_Flag = FALSE;

	return TRUE;
}

/** @brief  : Controller using Stanley algorithm
**  @agr    : current pose of the robot and Pathx, Pathy
**  @retval : Steering angle
**/
void Stanley_Follow(Stanley_t *stanley,
					double x_current, double y_current,
					double heading, double vel_robot) {
	double dx, dy, d;
	int32_t index_nearest = 0;
	int32_t upper_bound, lower_bound;
	double L = 0.1, Lf = 0; // L is distance from (Xc, Yc) to the front wheel
	/* Searching the nearest point */
	// Before 5 point -- Current Point -- New 5 Point
	if (stanley->refPointIndex == -1) {
		lower_bound = 0;
		upper_bound = stanley->NbOfWayPoints;
	} else {
		lower_bound = Max(0, stanley->refPointIndex - SEARCH_OFFSET);
		upper_bound = Min(stanley->NbOfWayPoints, stanley->refPointIndex + SEARCH_OFFSET);
	}
	// Compute distance and choose the smallest one
	for(int32_t i = lower_bound; i < upper_bound; ++i) {
		dx = x_current - stanley->P_X[i];
		dy = y_current - stanley->P_Y[i];
		d  = sqrt(dx*dx + dy*dy);

		if(i == lower_bound) {
			// Assume the first is the smallest
			stanley->dmin = d;
			index_nearest = i;
		} else if(stanley->dmin > d) {
			stanley->dmin = d;
			index_nearest = i;	// position of the minimum value
		}
	}

	Lf = stanley->K * vel_robot + 0.1;

	while((Lf > L) && (index_nearest < stanley->NbOfWayPoints - 1)) {
		dx 	= x_current - stanley->P_X[index_nearest];
		dy 	= y_current - stanley->P_Y[index_nearest];
		L 	= sqrt(dx*dx + dy*dy);
		index_nearest++;
	}
	// Slide ref point
	if( index_nearest > stanley->refPointIndex )
		stanley->refPointIndex = index_nearest;
	// Distance to begin and goal
	double dx_test, dy_test;
	dx_test = x_current - stanley->P_X[stanley->NbOfWayPoints - 1];
	dy_test = y_current - stanley->P_Y[stanley->NbOfWayPoints - 1];
	stanley->goal_radius = sqrt(dx_test*dx_test + dy_test*dy_test);

	Point_t point_current;
	point_current.x = x_current;
	point_current.y = y_current;
	// (-) |BEGIN >>>> (+) >>> |GOAL (-)
	stanley->distance_begin_line = lineDis2Point(myStanley.line_begin,
												point_current,
												myStanley.point_goal);
	stanley->distance_goal_line = lineDis2Point(myStanley.line_goal,
												point_current,
												myStanley.point_begin);
	// Check goal score
	if (stanley->goal_radius < 1 && stanley->distance_goal_line <= 0)
		stanley->Goal_Flag = TRUE;
	// Update control law
	stanley->efa = (x_current - stanley->P_X[stanley->refPointIndex]) * -cos(heading + PI/2)
			+ (y_current - stanley->P_Y[stanley->refPointIndex]) * -sin(heading + PI/2);

	stanley->Thetae = Pi_To_Pi(stanley->P_Yaw[stanley->refPointIndex] - heading);

	if (vel_robot < 0)
		vel_robot = 0;
	stanley->Thetad = atan2( (stanley->K) * (stanley->efa) , (vel_robot + stanley->Ksoft));

	stanley->Delta_Angle  = Pi_To_Pi(stanley->Thetae + stanley->Thetad);
	// Saturation
	if(stanley->Delta_Angle > PI)
		stanley->Delta_Angle = PI;
	else if(stanley->Delta_Angle < -PI)
		stanley->Delta_Angle = -PI;
}

uint8_t		Stanley_IsFinish(Stanley_t *stanley) {
	return stanley->Goal_Flag;
}


/** @brief  : Stanley updates path yaw
**  @agr    : Stanley
**  @retval : none
**/
void Stanley_UpdateRefYaw(Stanley_t *stanley) {
	for(int i = 0; i < stanley->NbOfP - 1; i++) {
		stanley->P_Yaw[i] = atan2(stanley->P_Y[i + 1] - stanley->P_Y[i], stanley->P_X[i + 1] - stanley->P_X[i]);
	}
	stanley->P_Yaw[stanley->NbOfP - 1] = stanley->P_Yaw[stanley->NbOfP - 2];
}

uint8_t		Stanley_CheckRotateFlag(void) {
	return myStanley.flag_rotate_first;
}

void		Stanley_ClearRotateFlag(void) {
	myStanley.flag_rotate_first = FALSE;
}
