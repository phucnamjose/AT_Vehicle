/*
 * vehicle_overall.h
 *
 *  Created on: Oct 31, 2020
 *      Author: Dang Nam
 */

#ifndef INC_VEHICLE_OVERALL_H_
#define INC_VEHICLE_OVERALL_H_


#include "def_myself.h"

typedef enum{
	Veh_NoneError = 0,
}enum_Error;


typedef enum{
	None_Mode = 0,
	Auto_Mode,
	Manual_Mode,
	Calib_Mode,
	KeyBoard_Mode,
	Soft_Reset_Mode,
}enum_Mode;


typedef struct Status{
	enum_Status        IMU_FirstGetAngle;           // IMU first time received angle after turning on
	enum_Status        Veh_SampleVelo;              //
	enum_Status	       Veh_SampleState;             // Vehicle sample time finished
	enum_Status	       Veh_Send_Data;               // Vehicle send data time finished
	enum_Status	       Veh_Enable_SendData;         // Flag to ENABLE send data from vehicle, set/unset by F7/F8
	enum_Status	       Veh_Calib_Flag;              // Calibration IMU
	enum_Status	       Veh_Auto_Flag;               //
	enum_Status	       Veh_Avoid_Flag;              //
	enum_Status	       Veh_AvoidEnable;             //
	enum_Status        Veh_MapAvailable;            // Set when vehicle has a full map
	enum_Status	       GPS_DataValid;               // Set when new gps packet is none error
	enum_Status	       GPS_Start_Receive_PathCor;   // Starting receive map coordinate from C#
	enum_Status	       GPS_SelfUpdatePosition_Flag; //
} Status;


typedef	struct Vehicle
{
	double             Max_Velocity; // in RPM
	double             Manual_Velocity; // in RPM
	double             Auto_Velocity;
	double             Manual_Angle; // in degree
	double             Sensor_Angle;
	enum_Mode          Mode;
	enum_Error         Veh_Error;
	int	               SendData_Ind;
	/* Calibration variables */
	uint32_t           Distance;
	char               ManualCtrlKey;
} Vehicle;


#define	TIME_LIMIT_MOVE_REFRESH	1.5 // second








void Vehicle_Stop(void);
void Vehicle_Forward(void);
void Vehicle_Backward(void);
void Vehicle_RotLeft(void);
void Vehicle_RotRight(void);
void Vehicle_ForwardLeft(void);
void Vehicle_ForwardRight(void);
void Vehicle_BackwardLeft(void);
void Vehicle_BackwardRight(void);




#endif /* INC_VEHICLE_OVERALL_H_ */
