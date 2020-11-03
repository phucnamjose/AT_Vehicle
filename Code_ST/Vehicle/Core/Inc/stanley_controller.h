/*
 * stanley_controller.h
 *
 *  Created on: Nov 2, 2020
 *      Author: Dang Nam
 */

#ifndef INC_STANLEY_CONTROLLER_H_
#define INC_STANLEY_CONTROLLER_H_


#include "def_myself.h"


typedef struct GPS {
	/* Robot statictis */
//	double              CorX; // x-coordinate converted from gps lat/lon to UTM [m]
//	double              CorY; // y-coordinate converted from gps lat/lon to UTM [m]

	double				TargetX;
	double				TargetY;

	double				CorX; // x-coordinate from Lidar
	double				CorY; // y-coordinate from Lidar

	uint32_t			position_count;
	uint32_t			position_count_pre;
	double              currentPosX; // x current coordinate [m]
	double              currentPosY; // y current coordinate [m]
	double              wheelPosX;
	double              wheelPosY;
	double              goal_radius; // radius between goal and current position
	double              efa;
	/* Stanley control variables */
	double              heading_angle;
	double              Thetae;
	double              Thetad;
	double              Delta_Angle;
	double              K;
	double              Ksoft;
	double              Step;
	double              Robot_Velocity; // (Vr + Vl) / 2
	double              dmin;
#define MAX_NUM_COORDINATE 1000
	double              P_X[MAX_NUM_COORDINATE];  // map's coordinate-x in UTM
	double              P_Y[MAX_NUM_COORDINATE];  // map's coordinate-y in UTM
	double              P_Yaw[MAX_NUM_COORDINATE];
	/* GPS NEO M8P input coordinates */
	//double              Latitude;
	//double              Longitude;
	int                 NbOfWayPoints; // 100 points
	int                 NbOfP; // [0..99]
	int                 NewDataAvailable;
	int                 refPointIndex;
	enum_Status         Goal_Flag;
} GPS;




void        GPS_ParametersInit(GPS *pgps);
void        GPS_StanleyControl(GPS *pgps, double M1Velocity, double M2Velocity);
//void        GPS_PursuitControl(GPS *pgps, double M1Velocity, double M2Velocity);
//double      GPS_DMS_To_DD(double LL);
//double	  GPS_StringToLat(char *inputmessage);
//double      GPS_StringToLng(char *inputmessage);
void        GPS_LatLonToUTM(GPS *pgps);  //Get 2 values of lat-lon and update UTM coordiante to Corx and Cory
void        GPS_ClearPathBuffer(GPS *pgps);
void        GPS_UpdatePathYaw(GPS *pgps);
//void        GPS_SavePathCoordinateToFlash(GPS *pgps, FlashMemory *pflash);


void		GPS_GenerateRefBuff(GPS *pgps, double target_x, double target_y);
void		GPS_UpdateNewPosion(double x, double y, double yaw, uint32_t count);

#endif /* INC_STANLEY_CONTROLLER_H_ */
