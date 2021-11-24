#include "usr/user_config.h"

#ifdef APP_CABINET
#include "usr/Machine_Control/sys_parameter.h"
#include "usr/Machine_Control/app_cabinet_test.h"
#include "usr/Machine_Control/cmd/cmd_c_loop_ctrl.h"
#include "usr/Machine_Control/cmd/cmd_cabinet_test.h"
#include "usr/Machine_Control/cmd/BIM/cmd/cmd_bim_ctrl.h"
#include "usr/Machine_Control/cmd/BP3/cmd/cmd_bp3_ctrl.h"
#include "sys/injection.h"

#include "drv/gpio_mux.h"
#include "drv/pwm.h"


void app_cabinet_test_init(void)
{
	//cmd_cramb_register();
	cmd_cabinet_register();
	cmd_c_loop_register();
	cmd_bim_register();
	cmd_bp3_register();
	// Configure GPIO mux
	// 0: top port on AMDC
	// GPIO_MUX_DEVICE1: Eddy current I/O IP block in the FPGA
	gpio_mux_set_device(1, GPIO_MUX_DEVICE1);

	// Turn on Kaman eddy current sensor digital interface
	eddy_current_sensor_init();
	eddy_current_sensor_enable();

	// AMDS
	gpio_mux_set_device(0, GPIO_MUX_DEVICE2);
	pwm_set_deadtime_ns(PWM_DEADTIME_NS);

}

#endif //APP_CABINET
