#include "usr/user_config.h"

#ifdef APP_CABINET

#include "usr/Cabinet_test/cmd/cmd_cabinet_test.h"
#include "usr/Cabinet_test/OpenloopVSI.h"
#include "usr/Cabinet_test/task_cabinet.h"
#include "usr/Cabinet_test/bearing_control.h"
#include "usr/Cabinet_test/current_control.h"
#include "usr/Cabinet_test/cabinet.h"
#include "usr/Cabinet_test/sensor_input.h"
#include "usr/Cabinet_test/mb_sensor.h"
#include "sys/defines.h"
#include "sys/commands.h"
#include "sys/debug.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "drv/pwm.h"
#include "drv/analog.h"

#define TS	(1.0 / TASK_CABINET_UPDATES_PER_SEC)// sample time

static command_entry_t cmd_entry;

#define NUM_HELP_ENTRIES	(16)
static command_help_t cmd_help[NUM_HELP_ENTRIES] = {
		{"setup", "Run Initial Setup"},
		{"init_cb", "Start Callback Loop"},
		{"deinit_cb", "Stop Callback Loop"},
		/*{"enable_cc", "Turn on Current Control for Specific Inverter"},
		{"dis_cc", "Turn off Current Control for Specific Inverter"},
		{"dis_cc_all", "Turn off Current Control for All Inverters"},*/
		{"set_vdc", "Set DC Bus for a Specific Inverter"},
		{"set_vdc_all", "Set DC Bus for All Inverters"},
		/*{"set_cur_3", "Set I1 and I2 currents on Inverter"},
		{"set_cur_1", "Set Current on Single Phase Inverter"},*/
		/*{"read_cur_3", "Print Instantaneous Three Phase Currents to Screen"},
		{"read_cur_1", "Print Instantaneous Single Phase Current to Screen"},
		{"cc_gain_3","Set Current Controller Gains for 3 Phase Inverter"},
		{"cc_gain_1", "Set Current Controller Gains for Single Phase Inverter"},
		{"cc_sen_tau", "set current sensor filter time constant"},*/
		
		{"pv3", "Set Pole Volts of 3 Phase Inverter"},
		{"pv1", "Set Pole Volts of 1 Phase Inverter"},
		{"phv3", "Set Phase Volts of 3 Phase Inverter"},
		{"phv1", "Set Phase Volts of 1 Phase Inverter"},
		{"openloop_vsi_3_enable", "Open Loop VSI enable"},
		{"openloop_vsi_3_disable", "Open Loop VSI disable"},
		{"openloop_vsi_3", "Open Loop VSI of 3 Phase Inverter"},
		{"read_adc_3", "Read ADC for 3 Phase Inverter"},
		{"read_cur_3", "Print Instantaneous Three Phase Currents to Screen"},
		{"read_mb_adc_3", "Read AMDS ADC for 3 Phase Inverter"},
		{"read_mb_cur_3", "Print Instantaneous  AMDS Three Phase Currents to Screen"}
		

};

void cmd_cabinet_register(void)
{
	OpenLoop_Command *OpenLoop;
	OpenLoop = &VSI_Openloop_command;
	default_inverter_setup(0);
	init_OpenLoop_Command(OpenLoop);
	// Populate the command entry block (struct)
	commands_cmd_init(&cmd_entry,
			"cabinet", "Power Electronics Cabinet Commands",
			cmd_help, NUM_HELP_ENTRIES,
			cmd_cabinet
	);

	// Register the command
	commands_cmd_register(&cmd_entry);
}

//
// Handles the 'cabinet' command
// and all sub-commands
//
int cmd_cabinet(int argc, char **argv)
{
	if (strcmp("setup", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 3) return CMD_INVALID_ARGUMENTS;

		double Vdc;
		Vdc = strtod(argv[2], NULL);

		cabinet_setup(Vdc, TS);
		//CRAMB_setup(get_CRAMB_ctxt());

		return CMD_SUCCESS;
	}

	if (strcmp("init_cb", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 2) return CMD_INVALID_ARGUMENTS;

		// Make sure cabinet task was not already inited
		if (task_cabinet_is_inited()) return CMD_FAILURE;

		task_cabinet_init();
		return CMD_SUCCESS;
	}

	if (strcmp("deinit_cb", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 2) return CMD_INVALID_ARGUMENTS;

		// Make sure cramb task was already inited
		if (!task_cabinet_is_inited()) return CMD_FAILURE;

		task_cabinet_deinit();
		return CMD_SUCCESS;
	}


	if (strcmp("set_vdc", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 4) return CMD_INVALID_ARGUMENTS;

		//read in arguments
		int inverter = atoi(argv[2]);
		double Vdc = strtod(argv[3], NULL);

		if (inverter <=0 || inverter >= CABINET_NUM_INVERTERS){
			return CMD_FAILURE;
		} else if (inverter == 1 || inverter == 4){
			inverter = find_invIdex_invID(inverter);
			if (inverter == -1)
			{
				return CMD_FAILURE;
			}
			Current_Controller_SinglePhase_t *cc_1 = get_single_phase_cc(inverter - 1);
			cc_1->inverter->Vdc = Vdc;

		} else{
			inverter = find_invIdex_invID(inverter);
			if (inverter == -1)
			{
				return CMD_FAILURE;
			}
			Current_Controller_ThreePhase_t *cc_3 = get_three_phase_cc(inverter - 5);
			cc_3->inverter->Vdc = Vdc;

		} 
		return CMD_SUCCESS;
	}

	if (strcmp("set_vdc_all", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 3) return CMD_INVALID_ARGUMENTS;

		//read in arguments
		double Vdc = strtod(argv[2], NULL);

		set_Vdc_all_inverters(Vdc);

		return CMD_SUCCESS;
	}

	

	if (strcmp("read_cur_3", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 3) return CMD_INVALID_ARGUMENTS;

		int inverter = atoi(argv[2]);
		inverter = find_invIdex_invID(inverter);
		if (inverter == -1)
		{
			return CMD_FAILURE;
		}
		Current_Controller_ThreePhase_t *cc_3 = get_three_phase_cc(inverter);

		double Iabc[3];
		input_read_currents_three_phase_abc(Iabc, cc_3->inverter);

		debug_printf("%f\n\r", Iabc[0]);
		debug_printf("%f\n\r", Iabc[1]);
		debug_printf("%f\n\n\r", Iabc[2]);

		return CMD_SUCCESS;
	}

	
	if (strcmp("read_cur_1", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 3) return CMD_INVALID_ARGUMENTS;

		int inverter = atoi(argv[2]);
		inverter = find_invIdex_invID(inverter);
		if (inverter == -1)
		{
			return CMD_FAILURE;
		}
		Current_Controller_SinglePhase_t *cc_1 = get_single_phase_cc(inverter);

		double I;
		input_read_current_single_phase(&I, cc_1->inverter);

		debug_printf("%f\n\r", I);

		return CMD_SUCCESS;
	}

	

	if (strcmp("pv3", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 6) return CMD_INVALID_ARGUMENTS;

		int inverter = atoi(argv[2]);
		inverter = find_invIdex_invID(inverter);
		if (inverter == -1)
		{
			return CMD_FAILURE;
		}
		double Va, Vb, Vc;
		Va = strtod(argv[3], NULL);
		Vb = strtod(argv[4], NULL);
		Vc = strtod(argv[5], NULL);

		Current_Controller_ThreePhase_t *cc_3 = get_three_phase_cc(inverter);

		set_pole_volts_three_phase(Va, Vb, Vc, cc_3->inverter);

		return CMD_SUCCESS;
	}


	if (strcmp("phv3", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 6) return CMD_INVALID_ARGUMENTS;

		int inverter = atoi(argv[2]);
		inverter = find_invIdex_invID(inverter);
		if (inverter == -1)
		{
			return CMD_FAILURE;
		}
		double Va, Vb, Vc;
		Va = strtod(argv[3], NULL);
		Vb = strtod(argv[4], NULL);
		Vc = strtod(argv[5], NULL);

		Current_Controller_ThreePhase_t *cc_3 = get_three_phase_cc(inverter);

		set_line_volts_three_phase(Va, Vb, Vc, cc_3->inverter);

		return CMD_SUCCESS;
	}


	if (strcmp("pv1", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 5) return CMD_INVALID_ARGUMENTS;

		int inverter = atoi(argv[2]);
		inverter = find_invIdex_invID(inverter);
		if (inverter == -1)
		{
			return CMD_FAILURE;
		}
		double Va, Vb;
		Va = strtod(argv[3], NULL);
		Vb = strtod(argv[4], NULL);

		Current_Controller_SinglePhase_t *cc_1 = get_single_phase_cc(inverter);

		set_pole_volts_single_phase(Va, Vb, cc_1->inverter);

		return CMD_SUCCESS;
	}

	if (strcmp("phv1", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 4) return CMD_INVALID_ARGUMENTS;

		int inverter = atoi(argv[2]);
		inverter = find_invIdex_invID(inverter);
		if (inverter == -1)
		{
			return CMD_FAILURE;
		}
		double V;
		V = strtod(argv[3], NULL);

		Current_Controller_SinglePhase_t *cc_1 = get_single_phase_cc(inverter);

		set_line_volts_single_phase(V, cc_1->inverter);

		return CMD_SUCCESS;
	}

	// Handle 'read_adc' sub-command
	if (strcmp("read_adc_3", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 3) return CMD_INVALID_ARGUMENTS;


		int inverter = atoi(argv[2]);
		inverter = find_invIdex_invID(inverter);
		if (inverter == -1)
		{
			return CMD_FAILURE;
		}
		Current_Controller_ThreePhase_t *cc_3 = get_three_phase_cc(inverter);


		double a = read_adc(cc_3->inverter->HW->sensor.Ia.adcCh);
		double b = read_adc(cc_3->inverter->HW->sensor.Ib.adcCh);
		double c = read_adc(cc_3->inverter->HW->sensor.Ic.adcCh);

		debug_printf("%f\n\r", a);
		debug_printf("%f\n\r", b);
		debug_printf("%f\n\n\r", c);

		return CMD_SUCCESS;
	}


	if (strcmp("read_mb_adc_3", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 3) return CMD_INVALID_ARGUMENTS;


		int inverter = atoi(argv[2]);
		inverter = find_invIdex_invID(inverter);
		if (inverter == -1)
		{
			return CMD_FAILURE;
		}
		InverterThreePhase_t *inv = get_three_phase_inverter(inverter);


		double a = read_mb_current_adc(inv->HW->mb_csensor.mb_Ia.mbCh);
		double b = read_mb_current_adc(inv->HW->mb_csensor.mb_Ib.mbCh);
		double c = read_mb_current_adc(inv->HW->mb_csensor.mb_Ic.mbCh);

		debug_printf("%f\n\r", a);
		debug_printf("%f\n\r", b);
		debug_printf("%f\n\n\r", c);

		return CMD_SUCCESS;
	}

	if (strcmp("read_mb_cur_3", argv[1]) == 0) {
			// Check correct number of arguments
			if (argc != 3) return CMD_INVALID_ARGUMENTS;

			int inverter = atoi(argv[2]);
			inverter = find_invIdex_invID(inverter);
			if (inverter == -1)
			{
				return CMD_FAILURE;
			}
			InverterThreePhase_t *inv = get_three_phase_inverter(inverter);

			double Iabc[3];
			input_read_mb_currents_three_phase_abc(&Iabc[0], inv);

			debug_printf("%f\n\r", Iabc[0]);
			debug_printf("%f\n\r", Iabc[1]);
			debug_printf("%f\n\n\r", Iabc[2]);

			return CMD_SUCCESS;
		}


	// Handle 'read_adc' sub-command
	if (strcmp("read_adc_1", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 3) return CMD_INVALID_ARGUMENTS;


		int inverter = atoi(argv[2]);
		inverter = find_invIdex_invID(inverter);
		if (inverter == -1)
		{
			return CMD_FAILURE;
		}
		Current_Controller_SinglePhase_t *cc_1 = get_single_phase_cc(inverter);

		double a = read_adc(cc_1->inverter->HW->sensor.Iz.adcCh);

		debug_printf("%f\n\r", a);

		return CMD_SUCCESS;
	}

    if (argc >= 2 && strcmp("stats", argv[1]) == 0) {
        if (argc == 3 && strcmp("print", argv[2]) == 0) {
            task_cabinet_stats_print();
            return SUCCESS;
        }

        if (argc == 3 && strcmp("reset", argv[2]) == 0) {
            task_cabinet_stats_reset();
            return SUCCESS;
        }
    }

	if (strcmp("openloop_vsi_3", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 5) return CMD_INVALID_ARGUMENTS;

		int inverter = atoi(argv[2]);
		inverter = find_invIdex_invID(inverter);
		if (inverter == -1)
		{
			return CMD_FAILURE;
		}
		//double freq, amp;
		VSI_Openloop_command.Num_inv = inverter;
		VSI_Openloop_command.freq = strtod(argv[3], NULL);
		VSI_Openloop_command.amp = strtod(argv[4], NULL);

		return CMD_SUCCESS;
	}

	if (strcmp("openloop_vsi_3_enable", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 3) return CMD_INVALID_ARGUMENTS;

		int inverter = atoi(argv[2]);
		inverter = find_invIdex_invID(inverter);
		if (inverter == -1)
		{
			return CMD_FAILURE;
		}
		//double freq, amp;
		VSI_Openloop_command.Num_inv = inverter;
		VSI_Openloop_command.enable = 1;

		return CMD_SUCCESS;
	}

	if (strcmp("openloop_vsi_3_disable", argv[1]) == 0) {
		// Check correct number of arguments
		if (argc != 3) return CMD_INVALID_ARGUMENTS;

		int inverter = atoi(argv[2]);
		inverter = find_invIdex_invID(inverter);
		if (inverter == -1)
		{
			return CMD_FAILURE;
		}
		//double freq, amp;
		VSI_Openloop_command.Num_inv = inverter;
		VSI_Openloop_command.enable = 0;

		return CMD_SUCCESS;
	}

	return CMD_INVALID_ARGUMENTS;
}

#endif //APP_CABINET






