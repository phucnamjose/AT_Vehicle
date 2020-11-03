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

GPS             GPS_NEO;
extern IMU 		Mag;

/*
** @brief  : Initial value for GPS functions
**  @agr    : input
**  @retval : Return fix value
**/
void GPS_ParametersInit(GPS *pgps)
{
	pgps->Goal_Flag = Check_NOK;
	//pgps->GPS_Error = Veh_NoneError;
	pgps->refPointIndex = -1; // there is no point that was referred
	pgps->K = 0.5;
	pgps->Ksoft = 0.08;
	pgps->Step = 0.5;
}

/** @brief  : GPS updates path yaw
**  @agr    : GPS
**  @retval : none
**/
void GPS_UpdatePathYaw(GPS *pgps)
{
	for(int i = 0; i < pgps->NbOfP; i++)
	{
		pgps->P_Yaw[i] = atan2(pgps->P_Y[i + 1] - pgps->P_Y[i], pgps->P_X[i + 1] - pgps->P_X[i]);
	}
	pgps->P_Yaw[pgps->NbOfP] = pgps->P_Yaw[pgps->NbOfP - 1];
}

/** @brief  : Save GPS path coordinate to internal flash memory
**  @agr    : GPS and FlashMemory
**  @retval : none
**/
//void GPS_SavePathCoordinateToFlash(GPS *pgps, FlashMemory *pflash)
//{
//	pflash->Length = 0;
//	pflash->Length += ToChar(pgps->NbOfWayPoints,&pflash->WriteInBuffer[pflash->Length],1);
//	pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
//	for(int i = 0; i < pgps->NbOfWayPoints; i++)
//	{
//		pflash->Length += ToChar(pgps->P_X[i], &pflash->WriteInBuffer[pflash->Length],6);
//		pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
//		pflash->Length += ToChar(pgps->P_Y[i], &pflash->WriteInBuffer[pflash->Length],6);
//		pflash->WriteInBuffer[pflash->Length++] = (uint8_t)',';
//	}
//	EraseMemory(FLASH_Sector_6);
//	WriteToFlash(pflash,FLASH_Sector_6,FLASH_GPSPara_BaseAddr);
//}

/** @brief  : Convert lattitude/longtitude in DMS format into DD format
              dd.ff = dd + mm/60 + ss/3600
**  @agr    : Lattitude/Longitude in NMEA format (DMS)
**  @retval : Lattitude/Longitude in DD format (*-1 for W and S)
**/
//double GPS_DMS_To_DD(double LL)
//{
//    double dd, mm;
//	dd = (int)(LL / 100);
//	mm = LL - (double)(dd * 100);
//	return (dd + mm/60);
//}

/** @brief  : Function get lat lon value from GNGLL message
**  @agr    :
        @pgps: pointer to instance of GPS
        @inputmessage: a string represent latitude in ddmm.mmmmm format
**  @retval : Value
**/
//double GPS_StringToLat(char *inputmessage)
//{
//	double s1 = 0, s2 = 0;
//	int temp = 1000;
//	for(int i = 0; i < 4; i++)
//	{
//		s1 += (inputmessage[i] - 48) * temp;
//		temp /= 10;
//	}
//	temp = 10000;
//	for(int i = 5; i < 10; i++)
//	{
//		s2 += (inputmessage[i] - 48) * temp;
//		temp /= 10;
//	}
//	s2 /= 100000;
//	return GPS_DMS_To_DD(s1 + s2);
//}

/** @brief  : Function get lat lon value from GNGLL message
**  @agr    : String value received from message
        @pgps: pointer to instance of GPS
        @inputmessage: a string represent latitude in dddmm.mmmmm format
**  @retval : Value
**/
//double GPS_StringToLng(char *inputmessage)
//{
//	double s1 = 0, s2 = 0;
//	int temp = 10000;
//	for(int i = 0; i < 5; i++)
//	{
//		s1 += (inputmessage[i] - 48) * temp;
//		s2 += (inputmessage[i + 6] - 48) * temp;
//		temp /= 10;
//	}
//	s2 /= 100000;
//	return GPS_DMS_To_DD(s1 + s2);
//}

void GPS_ClearPathBuffer(GPS *pgps)
{
	for(int i = 0; i < MAX_NUM_COORDINATE; ++i)
	{
		pgps->P_X[i] = pgps->P_Y[i] = pgps->P_Yaw[i] = 0;
	}
}

/** @brief  : Controller using Stanley algorithm
**  @agr    : current pose of the robot and Pathx, Pathy
**  @retval : Steering angle
**/
void GPS_StanleyControl(GPS *pgps, double v1_rpm, double v2_rpm)
{
	double dx, dy, d;
	int index = 0, i, upper_bound, lower_bound;
	double v1_mps, v2_mps;
	double L = 0.19, Lf = 0; // L is distance from (Xc, Yc) to the front wheel

	pgps->heading_angle = Pi_To_Pi(-Mag.Angle * (double)PI/180 + PI); // heading angle of vehicle [rad] [-pi, pi]
	v1_mps = WHEEL_RADIUS * 2 * PI * v1_rpm / 60; // [m/s]
	v2_mps = WHEEL_RADIUS * 2 * PI * v2_rpm / 60; // [m/s]
	pgps->Robot_Velocity = (v1_mps + v2_mps)/2;
	pgps->Robot_Velocity = (pgps->Robot_Velocity < 0) ? -pgps->Robot_Velocity : pgps->Robot_Velocity;

	// Calculate new Pos if there is no new data from GPS
	if(!pgps->NewDataAvailable)
	{
		//pgps->currentPosX += pgps->Robot_Velocity * cos(pgps->heading_angle) * Timer.T;
		//pgps->currentPosY += pgps->Robot_Velocity * sin(pgps->heading_angle) * Timer.T;
	}
	// Calculate the front wheel position
	pgps->wheelPosX = pgps->currentPosX + DISTANCE_BETWEEN_GPS_FRONT_WHEEL * cos(pgps->heading_angle);
	pgps->wheelPosY = pgps->currentPosY + DISTANCE_BETWEEN_GPS_FRONT_WHEEL * sin(pgps->heading_angle);

	//Searching the nearest point
	if (pgps->refPointIndex == -1)
	{
		lower_bound = 0;
		upper_bound = pgps->NbOfWayPoints;
	}
	else
	{
		// TODO: search la gi
		//lower_bound = Max(0, pgps->refPointIndex - SEARCH_OFFSET);
		//upper_bound = Min(pgps->NbOfWayPoints, pgps->refPointIndex + SEARCH_OFFSET);
	}

	for(i = lower_bound; i < upper_bound; ++i)
	{
		dx = pgps->wheelPosX - pgps->P_X[i];
		dy = pgps->wheelPosY - pgps->P_Y[i];
		d  = sqrt(dx*dx + dy*dy);

		if(i == lower_bound)
		{
			pgps->dmin = d;
			index = i;
		}
		else if(pgps->dmin > d)
		{
			pgps->dmin = d;
			index = i;	// position of the minimum value
		}
	}

	Lf = pgps->K * pgps->Robot_Velocity + 0.1;
	while((Lf > L) && (index < pgps->NbOfWayPoints - 1))
	{
		dx = pgps->wheelPosX - pgps->P_X[index];
		dy = pgps->wheelPosY - pgps->P_Y[index];
		L = sqrt(dx*dx + dy*dy);
		index++;
	}

	if( index > pgps->refPointIndex )
		pgps->refPointIndex = index;


	pgps->goal_radius = sqrt(pow(pgps->wheelPosX - pgps->P_X[pgps->NbOfWayPoints - 1], 2) +
						pow(pgps->wheelPosY - pgps->P_Y[pgps->NbOfWayPoints - 1], 2));
//
//	if( !VehStt.Veh_Avoid_Flag )
//	{
//		pgps->efa = -((pgps->wheelPosX - pgps->P_X[pgps->refPointIndex]) * cos(pgps->heading_angle + PI/2) +
//				(pgps->wheelPosY - pgps->P_Y[pgps->refPointIndex]) * sin(pgps->heading_angle + PI/2));
//
//		pgps->Thetae = Pi_To_Pi(pgps->heading_angle - pgps->P_Yaw[pgps->refPointIndex]); // [-pi, pi]
//		pgps->Thetad = -atan2( (pgps->K) * (pgps->efa) , (pgps->Robot_Velocity + pgps->Ksoft)); // [-pi, pi]
//		pgps->Delta_Angle  = (pgps->Thetae + pgps->Thetad)*(double)180/PI; // [-180, 180]
//		if(pgps->Delta_Angle > 160)
//			pgps->Delta_Angle = 160;
//		else if(pgps->Delta_Angle < -160)
//			pgps->Delta_Angle = -160;
//	}
}




void		GPS_GenerateRefBuff(GPS *pgps, double target_x, double target_y) {

}

