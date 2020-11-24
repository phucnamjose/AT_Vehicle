/*
 * debug_cmd.c
 *
 *  Created on: Oct 26, 2020
 *      Author: Dang Nam
 */

#include "debug_cmd.h"
#include "def_myself.h"
#include "stdio.h"
#include "string.h"

const char *DebugCmd_Code[]  = {"SETSPT", // 1
								"SETPID", // 2
								"START2", // 3
								"SSTOP2", // 4
								"SAVPID", // 5
								"GETPID", // 6
								"RSTPID", // 7
								"MOVVEH", // 8
								"REPONN", // 9
								"REPOFF", // 10
								"OUTPUT", // 11
								"SETHAN", // 12
								"GETSAM", // 13
								"RSTODO", // 14
								"CPYLID", // 15
								"MODEVE", // 16
								"ATMOVE", // 17
								"SETSPD", // 18
								"EMERGE", // 19
								"HANDAT", // 20
								"ATSTAR", // 21
								"ATSTOP", // 22
								"ATROTA"}; // 23

/* Implementation*/

enum_DebugCmd MsgToCmd(char *message, mainTaskMail_t *arguments) {
	char command[10];
	char para[70];
	int32_t lenght;
	memset(para, 0, 70*sizeof(char));

	// Separate cmd code and argument
	sscanf((char*)message, "%s %70c", command, para);

	// Compare DebugCmd_Code

		// 1. Set setpoint
	if ( 0 == strcmp( command, DebugCmd_Code[SET_SETPOINT])) {
		lenght = sscanf(para, "%lf %lf",
						&(arguments->setpoint_right),
						&(arguments->setpoint_left));
		if (lenght == 2) {
			arguments->cmd_code = SET_SETPOINT;
			return SET_SETPOINT;
		} else
			return CMD_NONE;

		// 2. Set PID current factor
	} else if ( 0 == strcmp(command, DebugCmd_Code[SET_PID])) {
		lenght = sscanf(para, "%lf %lf %lf %lf %lf %lf",
						&(arguments->R_kp),
						&(arguments->R_ki),
						&(arguments->R_kd),
						&(arguments->L_kp),
						&(arguments->L_ki),
						&(arguments->L_kd));
		if (lenght == 6) {
			arguments->cmd_code = SET_PID;
			return SET_PID;
		} else
			return CMD_NONE;
		// 3. Start Execute 2 Wheel
	} else if ( 0 == strcmp(command, DebugCmd_Code[START])) {
		arguments->cmd_code = START;
		return START;
		// 4. Stop Execute 2 Wheel
	} else if ( 0 == strcmp(command, DebugCmd_Code[STOP])) {
		arguments->cmd_code = STOP;
		return STOP;
		// 5. Save PID factor to Internal Flash
	} else if ( 0 == strcmp(command, DebugCmd_Code[SAVE_PID])) {
		arguments->cmd_code = SAVE_PID;
		return SAVE_PID;
		// 6. Send PID factor to PC
	} else if ( 0 == strcmp(command, DebugCmd_Code[GET_PID])) {
		arguments->cmd_code = GET_PID;
		return GET_PID;

		// 7. RESET PID
	}else if ( 0 == strcmp(command, DebugCmd_Code[RESET_PID])) {
		arguments->cmd_code = RESET_PID;
		return RESET_PID;

		// 8. Move vehicle
	} else if ( 0 == strcmp(command, DebugCmd_Code[MOVE_MANUAL])) {
		lenght = sscanf(para, "%d", (int *)&(arguments->move));
		if (lenght == 1) {
			arguments->cmd_code = MOVE_MANUAL;
			return MOVE_MANUAL;
		} else
			return CMD_NONE;
		// 9. Turn on report velocity
	} else if ( 0 == strcmp(command, DebugCmd_Code[REPORT_ON])) {
		arguments->cmd_code = REPORT_ON;
		return REPORT_ON;
		// 10. Turn off report velocity
	} else if ( 0 == strcmp(command, DebugCmd_Code[REPORT_OFF])) {
		arguments->cmd_code = REPORT_OFF;
		return REPORT_OFF;
		// 11. Set output 2 Wheel
	} else if ( 0 == strcmp(command, DebugCmd_Code[SET_OUTPUT])) {
		lenght = sscanf(para, "%lf %lf",
						&(arguments->R_output),
						&(arguments->L_output));
		if (lenght == 2) {
			arguments->cmd_code = SET_OUTPUT;
			return SET_OUTPUT;
		} else
			return CMD_NONE;
		// 12. Set hand manual
	}  else if ( 0 == strcmp(command, DebugCmd_Code[HAND_MANUAL])) {
		lenght = sscanf(para, "%d", (int *)&(arguments->hand_manual));
		if (lenght == 1) {
			arguments->cmd_code = HAND_MANUAL;
			return HAND_MANUAL;
		} else
			return CMD_NONE;
		// 13. Get sample headind
	} else if ( 0 == strcmp(command, DebugCmd_Code[GET_SAMPLE])) {
		arguments->cmd_code = GET_SAMPLE;
		return GET_SAMPLE;
		// 14. Reset odometry position
	} else if ( 0 == strcmp(command, DebugCmd_Code[RESET_ODOMETRY])) {
		arguments->cmd_code = RESET_ODOMETRY;
		return RESET_ODOMETRY;
		// 15. Copy lidar position to odometry position
	} else if ( 0 == strcmp(command, DebugCmd_Code[COPY_LIDAR_2_ODO])) {
		arguments->cmd_code = COPY_LIDAR_2_ODO;
		return COPY_LIDAR_2_ODO;
		// 16. Change mode vehicle
	} else if ( 0 == strcmp(command, DebugCmd_Code[MODE_VEHICLE])) {
		lenght = sscanf(para, "%d", (int *)&(arguments->mode_vehicle));
		if (lenght == 1) {
			arguments->cmd_code = MODE_VEHICLE;
			return MODE_VEHICLE;
		} else
			return CMD_NONE;
		// 17. Auto moving to desire point
	} else if ( 0 == strcmp(command, DebugCmd_Code[MOVE_AUTO])) {
		lenght = sscanf(para, "%lf %lf",
						&(arguments->target_x),
						&(arguments->target_y));
		if (lenght == 2) {
			arguments->cmd_code = MOVE_AUTO;
			return MOVE_AUTO;
		} else
			return CMD_NONE;
		// 18. Change speed move
	} else if ( 0 == strcmp(command, DebugCmd_Code[SPEED_MOVE])) {
		lenght = sscanf(para, "%lf",
						&(arguments->speed_move));
		if (lenght == 1) {
			arguments->cmd_code = SPEED_MOVE;
			return SPEED_MOVE;
		} else
			return CMD_NONE;
		// 19. Stop move and hand
	} else if ( 0 == strcmp(command, DebugCmd_Code[EMERGENCY])) {
		arguments->cmd_code = EMERGENCY;
		return EMERGENCY;
		// 20. Hand robot auto
	} else if ( 0 == strcmp(command, DebugCmd_Code[HAND_AUTO])) {
		lenght = sscanf(para, "%d", (int *)&(arguments->hand_auto));
		if (lenght == 1) {
			arguments->cmd_code = HAND_AUTO;
			return HAND_AUTO;
		} else
			return CMD_NONE;
		// 21. Start auto moving
	} else if ( 0 == strcmp(command, DebugCmd_Code[AUTO_START])) {
		arguments->cmd_code = AUTO_START;
		return AUTO_START;
		// 22. Stop auto moving
	} else if ( 0 == strcmp(command, DebugCmd_Code[AUTO_STOP])) {
		arguments->cmd_code = AUTO_STOP;
		return AUTO_STOP;
		// 23. Rotate forever
	} else if ( 0 == strcmp(command, DebugCmd_Code[AUTO_ROTATE])) {
		arguments->cmd_code = AUTO_ROTATE;
		return AUTO_ROTATE;

		// Wrong code
	} else {
		return CMD_NONE;
	}
}



