/*
 * vehicle_overall.h
 *
 *  Created on: Oct 31, 2020
 *      Author: Dang Nam
 */

#ifndef INC_VEHICLE_OVERALL_H_
#define INC_VEHICLE_OVERALL_H_


#include "def_myself.h"
#include "debug_cmd.h"

typedef enum{
	VEH_NONE_ERROR = 0,
	VEH_NO_LIDAR // No IMU, No Refference
}enum_Error;

typedef enum enum_AutoRunState {
	AUTO_RUN_IDLE,
	AUTO_RUN_NEW_TARGET,
	AUTO_RUN_WAIT_START,
	AUTO_RUN_MOVING,
	AUTO_RUN_FINISH,
	AUTO_RUN_ERROR
}enum_AutoRunState;

typedef	struct Vehicle_t
{
	double             	speed_manual;
	double             	speed_auto;
	enum_Error         	error_vehicle;
	/* Calibration variables */
	uint32_t           	Distance;
	/* Nam */
	enum_ModeVehicle	mode_vehicle;
	enum_MoveManual     move_manual;
	enum_AutoRunState	state_auto_run;
	double			   	time_out_move_manual;
	double				time_out_rotate_stable;
	Position_t			position_lidar;
	Position_t			position_center_veh;
	Position_t			position_odometry;
	double				target_x, target_y, target_yaw;
	uint8_t				target_frame[38];
	double				start_x, start_y;
	uint32_t			position_lidar_count;
	uint8_t				miss_lidar_count;

} Vehicle_t;


#define	TIME_LIMIT_MOVE_REFRESH	1.5 // (second)

void	Vehicle_Init();
void	Vehicle_ChangeMode(enum_ModeVehicle mode);
void	Vehicle_ChangeSpeed(double speed);

void	Vehicle_EstimatePosition(uint8_t has_lidar);
void	Vehicle_Odometry();

void 	Vehicle_AutoDrive(void);
void	Vehicle_TestFuzzy(void);
void	Vehicle_AutoNewTarget(double target_x, double target_y, uint8_t *target_data);
void	Vehicle_AutoStart(void);
void	Vehicle_AutoStop(void);
void	Vehicle_AutoRunState(void);

void 	Vehicle_Stop(void);
void 	Vehicle_Forward(void);
void 	Vehicle_Backward(void);
void 	Vehicle_RotLeft(void);
void 	Vehicle_RotRight(void);
void 	Vehicle_ForwardLeft(void);
void 	Vehicle_ForwardRight(void);
void 	Vehicle_BackwardLeft(void);
void 	Vehicle_BackwardRight(void);
void	Vehicle_CheckManualTimeOut(void);


#endif /* INC_VEHICLE_OVERALL_H_ */
