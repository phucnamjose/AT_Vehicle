/*
 * debug_cmd.h
 *
 *  Created on: Oct 26, 2020
 *      Author: Dang Nam
 */

#ifndef INC_DEBUG_CMD_H_
#define INC_DEBUG_CMD_H_

#include "def_myself.h"

/* Enum type define*/
typedef	enum enum_DebugCmd{
	CMD_NONE = -1,
	SET_SETPOINT,		// 1. Set reference
	SET_PID,			// 2. Set PID current factor
	START,				// 3. Start Execute 2 Wheel
	STOP,				// 4. Stop Execute 2 Wheel
	SAVE_PID,			// 5. Save PID factor to Internal Flash
	GET_PID,			// 6. Send PID factor to PC
	RESET_PID,			// 7. Reset full PID
	MOVE_MANUAL,		// 8. Move vehicle
	REPORT_ON,			// 9. Start send feedback
	REPORT_OFF,			// 10. Stop send feedback
	SET_OUTPUT,			// 11. Set output 2 Wheel
	HAND_MANUAL,		// 12. Hand robot manual
	GET_SAMPLE,			// 13. Read sample angular
	RESET_ODOMETRY,		// 14. Reset odometry intergrate
	COPY_LIDAR_2_ODO,	// 15. Copy lidar position to odometry position
	MODE_VEHICLE,		// 16. Change mode vehicle
	MOVE_AUTO,			// 17. Auto moving to desire point
	SPEED_MOVE,			// 18. Change speed move
	EMERGENCY,			// 19. Stop move and hand
	HAND_AUTO,			// 20. Hand robot auto
	AUTO_START,			// 21. Start auto moving
	AUTO_STOP,			// 22. Stop auto moving
	AUTO_ROTATE			// 23. Rotate forever
}enum_DebugCmd;

typedef	enum enum_MoveManual{
	STOP_VEHICLE = 0,
	FORWARD,
	BACKWARD,
	ROTATE_LEFT,
	ROTATE_RIGHT,
	FORWARD_LEFT,
	FORWARD_RIGHT,
	BACKWARD_LEFT,
	BACKWARD_RIGHT
}enum_MoveManual;

typedef enum enum_HandMode{
	HAND_MODE_HOME = 0,
	HAND_MODE_MANUAL,
	HAND_MODE_AUTO
}enum_HandMode;

typedef	enum enum_HandManual{
	HAND_MAN_STOP = 0,
	HAND_MAN_UP,
	HAND_MAN_DOWN,
	HAND_MAN_LEFT,
	HAND_MAN_RIGHT,
	HAND_MAN_NODE_UP,
	HAND_MAN_NODE_DOWN
}enum_HandManual;

typedef	enum enum_HandAuto{
	HAND_AUTO_NONE = 0,
	HAND_AUTO_RETANGLE,
	HAND_AUTO_CIRCLE
}enum_HandAuto;

typedef enum enum_ModeVehicle{
	MODE_NONE = 0,
	MODE_MANUAL,
	MODE_AUTO
}enum_ModeVehicle;

/* Object form*/
typedef struct mainTaskMail_t
{
	enum_DebugCmd 		cmd_code;
	double 				setpoint_right;
	double				setpoint_left;
	double 				R_ki, R_kp, R_kd;
	double 				L_ki, L_kp, L_kd;
	double				R_output, L_output;
	enum_MoveManual		move;
	double				target_x, target_y;
	uint8_t				target_frame[38];
	enum_HandManual		hand_manual;
	enum_HandAuto		hand_auto;
	enum_ModeVehicle	mode_vehicle;
	double				speed_move;
}mainTaskMail_t;

typedef struct Position_t
{
    double x;
    double y;
    double z;
    double yaw;
} Position_t;

/* Function Prototype */
enum_DebugCmd MsgToCmd(char *message, mainTaskMail_t *arguments);


#endif /* INC_DEBUG_CMD_H_ */
