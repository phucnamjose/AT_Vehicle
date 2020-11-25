/*
 * stanley_controller.h
 *
 *  Created on: Nov 2, 2020
 *      Author: Dang Nam
 */

#ifndef INC_STANLEY_CONTROLLER_H_
#define INC_STANLEY_CONTROLLER_H_


#include "def_myself.h"
#include "geometry.h"


#define STEP_REFER	(0.2)
#define SEARCH_OFFSET                    	(5) // for nearest point searching


typedef struct Stanley_t {
	/* Robot statictis */
	double              goal_radius; // radius between goal and current position
	double				distance_goal_line;
	double				distance_begin_line;
	Point_t				point_begin;
	Point_t				point_goal;
	Line_t				line_begin;
	Line_t				line_goal;
	double				angle_go;
	uint8_t				flag_rotate_first;
	/* Stanley_t control variables */
	double              efa;
	double              Thetae;
	double              Thetad;
	double              Delta_Angle;
	double              K;
	double              Ksoft;
	double              Step;
	double              dmin;
#define MAX_NUM_COORDINATE 1000
	double              P_X[MAX_NUM_COORDINATE];  // map's coordinate-x in UTM
	double              P_Y[MAX_NUM_COORDINATE];  // map's coordinate-y in UTM
	double              P_Yaw[MAX_NUM_COORDINATE];
	// Reference point
	int32_t             NbOfWayPoints; // 100 points
	int32_t             NbOfP; // [0..99]
	int32_t             refPointIndex;
	uint8_t         	Goal_Flag;
} Stanley_t;


/*  Function Prototypes*/
void        Stanley_Init(Stanley_t *stanley);
uint8_t		Stanley_InitNewPath(Stanley_t *stanley,
								double target_x, double target_y,
								double start_x, double start_y,
								double yaw);
void        Stanley_Follow(Stanley_t *stanley,
							double x_current, double y_current,
							double heading, double vel_robot);
uint8_t		Stanley_IsFinish(Stanley_t *stanley);
void        Stanley_UpdateRefYaw(Stanley_t *stanley);
uint8_t		Stanley_CheckRotateFlag(void);
void		Stanley_ClearRotateFlag(void);

#endif /* INC_STANLEY_CONTROLLER_H_ */
