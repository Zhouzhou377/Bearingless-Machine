#include "usr/user_config.h"

#ifdef APP_CABINET
#include "usr/Machine_Control/currentloop_control.h"
#include "usr/Machine_Control/cmd/cmd_c_loop_ctrl.h"
#include "usr/Machine_Control/task_cabinet.h"
#include "drv/cpu_timer.h"
#include "usr/Machine_Control/cabinet.h"
#include "usr/Machine_Control/analog_sensor.h"
#include "usr/Machine_Control/mb_sensor.h"
#include "sys/defines.h"
#include "sys/commands.h"
#include "sys/debug.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "drv/analog.h"

//currentloop_control c_loop_control;
//cmd_signal cmd_enable;

//#define TS	(1.0 / TASK_CABINET_UPDATES_PER_SEC)// sample time

static command_entry_t cmd_entry;

#define NUM_HELP_ENTRIES	(12)
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
		{"sel_config", "Select configuration"}, 
		{"enable_testloop", "Enable loop testing"},
};

void cmd_c_loop_register(void)
{
	
	// Populate the command entry block (struct)
	commands_cmd_init(&cmd_entry,
			"c_loop", "Twin Bearingless Control Commands",
			cmd_help, NUM_HELP_ENTRIES,
			cmd_c_loop
	);

	// Register the command
	commands_cmd_register(&cmd_entry);
}

//
// Handles the 'cabinet' command
// and all sub-commands
//
int cmd_c_loop(int argc, char **argv)
{

	if (strcmp("init", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 2) return CMD_INVALID_ARGUMENTS;

		// Make sure cabinet task was not already inited
		currentloop_control *c_loop = init_currentloop();
		
		return CMD_SUCCESS;
	}

	if (strcmp("deinit", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 2) return CMD_INVALID_ARGUMENTS;

		// Make sure cabinet task was not already inited
		currentloop_control *c_loop = deinit_currentloop();
		cmd_enable.enable_current_control = 1;
		cmd_enable.enable_openloop = 0;
		cmd_enable.enable_testloop = 0;
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

		c_loop_control.c_loop_inv1.inv->Vdc = Vdc;
		c_loop_control.c_loop_inv2.inv->Vdc = Vdc;
		c_loop_control.c_loop_inv3.inv->Vdc = Vdc;
		if(c_loop_control.sel_config == InvFour){
			c_loop_control.c_loop_inv4.inv->Vdc = Vdc;
		}
		return CMD_SUCCESS;
	}

	
	if (strcmp("enable_ctrl", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 2) return CMD_INVALID_ARGUMENTS;

		cmd_enable.enable_current_control = 1;
		cmd_enable.enable_openloop = 0;
		reset_regulator();
		return CMD_SUCCESS;
	}

	if (strcmp("enable_testloop", argv[1]) == 0){
		if (argc != 2) return CMD_INVALID_ARGUMENTS;
		cmd_enable.enable_testloop = 1;
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
		c_loop_control.sel_config = sel_config;
		
		return CMD_SUCCESS;
	}

	if (strcmp("set_trq", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 4) return CMD_INVALID_ARGUMENTS;

		//read in arguments
		double trq_d = strtod(argv[2], NULL);
		double trq_q = strtod(argv[3], NULL);

		if(c_loop_control.sel_config == InvFour){
			c_loop_control.tq.Idq0_ref[0] = trq_d/2.0;
			c_loop_control.tq.Idq0_ref[1] = trq_q/2.0;
			c_loop_control.tq.Idq0_ref[2] = 0.0;
			c_loop_control.tq2.Idq0_ref[0] = trq_d/2.0;
			c_loop_control.tq2.Idq0_ref[1] = trq_q/2.0;
			c_loop_control.tq2.Idq0_ref[2] = 0.0;
		}else{
			c_loop_control.tq.Idq0_ref[0] = trq_d;
			c_loop_control.tq.Idq0_ref[1] = trq_q;
			c_loop_control.tq.Idq0_ref[2] = 0.0;
		}
		return CMD_SUCCESS;
	}

	if (strcmp("set_s1", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 4) return CMD_INVALID_ARGUMENTS;

		//read in arguments
		double I_d = strtod(argv[2], NULL);
		double I_q = strtod(argv[3], NULL);

		c_loop_control.s1.Idq0_ref[0] = I_d;
		c_loop_control.s1.Idq0_ref[1] = I_q;
		c_loop_control.s1.Idq0_ref[2] = 0.0;
		return CMD_SUCCESS;
	}

	if (strcmp("set_s2", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 4) return CMD_INVALID_ARGUMENTS;

		//read in arguments
		double I_d = strtod(argv[2], NULL);
		double I_q = strtod(argv[3], NULL);

		c_loop_control.s2.Idq0_ref[0] = I_d;
		c_loop_control.s2.Idq0_ref[1] = I_q;
		c_loop_control.s2.Idq0_ref[2] = 0.0;
		return CMD_SUCCESS;
	}

	if (strcmp("set_freq", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 3) return CMD_INVALID_ARGUMENTS;

		//read in arguments
		double freq = strtod(argv[2], NULL);

		if(c_loop_control.sel_config == InvFour){
			c_loop_control.tq2.we = 2*PI*freq;
		}
		c_loop_control.tq.we = 2*PI*freq;
		c_loop_control.s1.we = -2*PI*freq;
		c_loop_control.s2.we = -2*PI*freq;
		return CMD_SUCCESS;
	}
	
	if (strcmp("disable_ctrl", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 2) return CMD_INVALID_ARGUMENTS;

		cmd_enable.enable_current_control = 0;
		reset_regulator();
		
		return CMD_SUCCESS;
	}

	return CMD_INVALID_ARGUMENTS;
}

#endif //APP_CABINET
