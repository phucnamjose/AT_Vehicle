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

#define VEH_MAX_MANUAL_FORWARD		0.5 	// m/s
#define VEH_MAX_MANUAL_BACKWARD		0.25 	// m/s
#define VEH_MAX_AUTO				0.2		// m/s

#define	TIME_LIMIT_MOVE_REFRESH	1.5 // (second)


typedef enum{
	VEH_NONE_ERROR = 0,
	VEH_NO_LIDAR // No IMU, No Refference
}enum_Error;

typedef enum enum_SpeedManual {
	PHASE_MAN_ACC,
	PHASE_MAN_DEC
}enum_PhaseManual;

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
	// Common
	double             	speed_manual;// percent
	double             	speed_auto;// percent
	enum_Error         	error_vehicle;
	enum_ModeVehicle	mode_vehicle;
	// Manual
	enum_MoveManual     move_manual;
	enum_MoveManual     pre_move_manual;
	enum_AutoRunState	state_auto_run;
	double			   	time_out_move_manual;
	double				time_vel_manual;
	enum_PhaseManual	phase_manual;
	double				speed_man_current;
	// Auto
	double				target_x, target_y, target_yaw;
	uint8_t				target_frame[38];
	double				start_x, start_y;
	double				speed_auto_current;
	uint8_t				count_auto;
	// Position
	double				time_out_rotate_stable;
	Position_t			position_lidar;
	Position_t			position_center_veh;
	Position_t			position_odometry;
	uint32_t			position_lidar_count;
	uint32_t			miss_lidar_count;
	uint8_t				miss_lidar_flag;
	uint8_t				count_lidar;
} Vehicle_t;

/* Function Prototypes*/
// Common
void	Vehicle_Init(void);
void	Vehicle_ChangeMode(enum_ModeVehicle mode);
void	Vehicle_ChangeSpeed(double speed);
void	Vehicle_Localization(void);
void	Vehicle_EstimatePosition(uint8_t has_lidar);
void	Vehicle_Odometry(void);
void	Vehicle_ResetOdometry(void);
void	Vehicle_CopyLidar(void);
// Auto
void 	Vehicle_AutoDrive(void);
void	Vehicle_TestFuzzy(void);
void	Vehicle_TestHeadPID(void);
void	Vehicle_AutoNewTarget(double target_x, double target_y, uint8_t *target_data);
void	Vehicle_AutoStart(void);
void	Vehicle_AutoStop(void);
void	Vehicle_AutoRunState(void);
// Manual
void	Vehicle_SetLinearVel(double long_right, double long_left);
void 	Vehicle_StopSoft(void);
void 	Vehicle_StopHard(void);
void 	Vehicle_Forward(void);
void 	Vehicle_Backward(void);
void 	Vehicle_RotLeft(void);
void 	Vehicle_RotRight(void);
void 	Vehicle_ForwardLeft(void);
void 	Vehicle_ForwardRight(void);
void 	Vehicle_BackwardLeft(void);
void 	Vehicle_BackwardRight(void);
void	Vehicle_ManualRun(void);


#endif /* INC_VEHICLE_OVERALL_H_ */
