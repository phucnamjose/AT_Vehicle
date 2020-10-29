/*
 * debug_cmd.h
 *
 *  Created on: Oct 26, 2020
 *      Author: Dang Nam
 */

#ifndef INC_DEBUG_CMD_H_
#define INC_DEBUG_CMD_H_

/* Enum type define*/
typedef	enum{
	CMD_NONE = -1,
	SET_SETPOINT,		// 1. Set reference
	SET_PID,			// 2. Set PID current factor
	START,				// 3. Start Execute 2 Wheel
	STOP,				// 4. Stop Execute 2 Wheel
	SAVE_PID,			// 5. Save PID factor to Internal Flash
	GET_PID,			// 6. Send PID factor to PC
	MOVE,				// 7. Move vehicle
	REPORT_ON,			// 8. Start send feedback
	REPORT_OFF,			// 9. Stop send feedback
	SET_OUTPUT,			// 10. Set output 2 Wheel
	HAND,				// 11. Hand robot
	SET_FEATURE,		// 12. Set feature
	POSITION,			// 13. Position information
	FORWARD_MSG			// 14. Forward to Pi or Mega
}enum_DebugCmd;

typedef	enum{
	STOP_VEHICLE = 0,
	FORWARD,
	BACKWARD,
	ROTATE_LEFT,
	ROTATE_RIGHT,
	FORWARD_LEFT,
	FORWARD_RIGHT,
	BACKWARD_LEFT,
	BACKWARD_RIGHT
}enum_Move;


typedef	enum{
	HAND_STOP = 0,
	HAND_LEFT,
	HAND_RIGHT,
	HAND_UP,
	HAND_DOWN,
	HAND_NODE_UP,
	HAND_NODE_DOWN
}enum_Hand;


/* Object form*/
typedef struct mainTaskMail_t
{
	enum_DebugCmd 	cmd_code;
	double 			setpoint_right;
	double			setpoint_left;
	double 			R_ki, R_kp, R_kd;
	double 			L_ki, L_kp, L_kd;
	double			R_output, L_output;
	enum_Move		move;
	enum_Hand		hand;
	double			x, y, z, yaw;
}mainTaskMail_t;

/* Function Prototype */
enum_DebugCmd MsgToCmd(char *message, mainTaskMail_t *arguments);


#endif /* INC_DEBUG_CMD_H_ */
