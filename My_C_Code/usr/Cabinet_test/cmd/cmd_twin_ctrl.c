#include "usr/user_config.h"

#ifdef APP_CABINET
#include "usr/Cabinet_test/twinbearingless_control.h"
#include "usr/Cabinet_test/cmd/cmd_twin_ctrl.h"
#include "usr/Cabinet_test/task_cabinet.h"

#include "usr/Cabinet_test/cabinet.h"
#include "usr/Cabinet_test/analog_sensor.h"
#include "usr/Cabinet_test/mb_sensor.h"
#include "sys/defines.h"
#include "sys/commands.h"
#include "sys/debug.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "drv/analog.h"

twinbearingless_control twin_control;
cmd_signal cmd_enable;

//#define TS	(1.0 / TASK_CABINET_UPDATES_PER_SEC)// sample time

static command_entry_t cmd_entry;

#define NUM_HELP_ENTRIES	(11)
static command_help_t cmd_help[NUM_HELP_ENTRIES] = {
		{"init", "Initialize control loop"},
		{"deinit", "Deinitialize control loop"},
		{"reset", "Reset regulator"},
		{"set_vdc", "Set DC Bus for a Specific Inverter"},
		{"enable_ctrl", "Enable current regulation"},
		{"set_trq", "Set torque current references dq0"},
		{"set_s1", "Set suspension 1 current references dq0"},
		{"set_s2", "Set suspension 2 current references dq0"},
		{"set_freq", "Set rotating frequency Hz"},
		{"disable_ctrl", "Enable current regulation"},
		{"sel_config", "Set rotating frequency Hz"},
};

void cmd_twin_register(void)
{
	
	// Populate the command entry block (struct)
	commands_cmd_init(&cmd_entry,
			"twin", "Twin Bearingless Control Commands",
			cmd_help, NUM_HELP_ENTRIES,
			cmd_twin
	);

	// Register the command
	commands_cmd_register(&cmd_entry);
}

//
// Handles the 'cabinet' command
// and all sub-commands
//
int cmd_twin(int argc, char **argv)
{

	if (strcmp("init", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 2) return CMD_INVALID_ARGUMENTS;

		// Make sure cabinet task was not already inited
		twinbearingless_control *twin = init_twinbearingless();
		
		return CMD_SUCCESS;
	}

	if (strcmp("deinit", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 2) return CMD_INVALID_ARGUMENTS;

		// Make sure cabinet task was not already inited
		twinbearingless_control *twin = deinit_twinbearingless();
		
		return CMD_SUCCESS;
	}

	if (strcmp("reset", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 2) return CMD_INVALID_ARGUMENTS;

		// Make sure cabinet task was not already inited
		reset_regulator();
		
		return CMD_SUCCESS;
	}


	if (strcmp("set_vdc", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 3) return CMD_INVALID_ARGUMENTS;

		//read in arguments
		double Vdc = strtod(argv[2], NULL);

		twin_control.twin_inv1.inv->Vdc = Vdc;
		twin_control.twin_inv2.inv->Vdc = Vdc;
		twin_control.twin_inv3.inv->Vdc = Vdc;
		if(twin_control.sel_config == InvFour){
			twin_control.twin_inv4.inv->Vdc = Vdc;
		}
		return CMD_SUCCESS;
	}

	
	if (strcmp("enable_ctrl", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 2) return CMD_INVALID_ARGUMENTS;

		cmd_enable.enable_currentcontrol = 1;
		cmd_enable.enable_openloop = 0;
		reset_regulator();
		return CMD_SUCCESS;
	}

	if (strcmp("enable_log", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 2) return CMD_INVALID_ARGUMENTS;

		cmd_enable.enable_log = 1;
		
		return CMD_SUCCESS;
	}

	if (strcmp("sel_config", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 3) return CMD_INVALID_ARGUMENTS;

		double sel_config = strtod(argv[2], NULL);
		twin_control.sel_config = sel_config;
		
		return CMD_SUCCESS;
	}

	if (strcmp("set_trq", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 4) return CMD_INVALID_ARGUMENTS;

		//read in arguments
		double trq_d = strtod(argv[2], NULL);
		double trq_q = strtod(argv[3], NULL);

		if(twin_control.sel_config == InvFour){
			twin_control.tq.Idq0_ref[0] = trq_d/2.0;
			twin_control.tq.Idq0_ref[1] = trq_q/2.0;
			twin_control.tq.Idq0_ref[2] = 0.0;
			twin_control.tq2.Idq0_ref[0] = trq_d/2.0;
			twin_control.tq2.Idq0_ref[1] = trq_q/2.0;
			twin_control.tq2.Idq0_ref[2] = 0.0;
		}else{
			twin_control.tq.Idq0_ref[0] = trq_d;
			twin_control.tq.Idq0_ref[1] = trq_q;
			twin_control.tq.Idq0_ref[2] = 0.0;
		}
		return CMD_SUCCESS;
	}

	if (strcmp("set_s1", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 4) return CMD_INVALID_ARGUMENTS;

		//read in arguments
		double I_d = strtod(argv[2], NULL);
		double I_q = strtod(argv[3], NULL);

		twin_control.s1.Idq0_ref[0] = I_d;
		twin_control.s1.Idq0_ref[1] = I_q;
		twin_control.s1.Idq0_ref[2] = 0.0;
		return CMD_SUCCESS;
	}

	if (strcmp("set_s2", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 4) return CMD_INVALID_ARGUMENTS;

		//read in arguments
		double I_d = strtod(argv[2], NULL);
		double I_q = strtod(argv[3], NULL);

		twin_control.s2.Idq0_ref[0] = I_d;
		twin_control.s2.Idq0_ref[1] = I_q;
		twin_control.s2.Idq0_ref[2] = 0.0;
		return CMD_SUCCESS;
	}

	if (strcmp("set_freq", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 3) return CMD_INVALID_ARGUMENTS;

		//read in arguments
		double freq = strtod(argv[2], NULL);


		twin_control.tq.we = 2*PI*freq;
		twin_control.s1.we = -2*PI*freq;
		twin_control.s2.we = -2*PI*freq;
		return CMD_SUCCESS;
	}
	
	if (strcmp("disable_ctrl", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 2) return CMD_INVALID_ARGUMENTS;

		cmd_enable.enable_currentcontrol = 0;
		reset_regulator();
		
		return CMD_SUCCESS;
	}

	return CMD_INVALID_ARGUMENTS;
}

#endif //APP_CABINET
