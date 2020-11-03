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

const char *DebugCmd_Code[]  = {"SETSPT",
								"SETPID",
								"START2",
								"SSTOP2",
								"SAVPID",
								"GETPID",
								"RSTPID",
								"MOVVEH",
								"REPONN",
								"REPOFF",
								"OUTPUT",
								"SETHAN",
								"POWONN",
								"POWOFF"};

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
	} else if ( 0 == strcmp(command, DebugCmd_Code[MOVE])) {
		lenght = sscanf(para, "%d", (int *)&(arguments->move));
		if (lenght == 1) {
			arguments->cmd_code = MOVE;
			return MOVE;
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
		// 12. Turn on motion power
	} else if ( 0 == strcmp(command, DebugCmd_Code[POWER_ON])) {
			arguments->cmd_code = POWER_ON;
			return POWER_ON;
		// 13. Turn off motion power
	} else if ( 0 == strcmp(command, DebugCmd_Code[POWER_OFF])) {
			arguments->cmd_code = POWER_OFF;
			return POWER_OFF;

		// Wrong code
	} else {
		return CMD_NONE;
	}
}



