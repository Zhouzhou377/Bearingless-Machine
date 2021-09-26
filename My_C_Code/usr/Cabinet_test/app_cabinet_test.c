#include "usr/user_config.h"

#ifdef APP_CABINET

#include "usr/Cabinet_test/app_cabinet_test.h"
#include "usr/Cabinet_test/cmd/cmd_twin_ctrl.h"
#include "usr/Cabinet_test/cmd/cmd_cabinet_test.h"
#include "sys/injection.h"

#include "drv/gpio_mux.h"

void app_cabinet_test_init(void)
{
	//cmd_cramb_register();
	cmd_cabinet_register();
	cmd_twin_register();
	cmd_BIM_register();
	// Configure GPIO mux
	// 0: top port on AMDC
	// GPIO_MUX_DEVICE1: Eddy current I/O IP block in the FPGA
	gpio_mux_set_device(1, GPIO_MUX_DEVICE1);

	// Turn on Kaman eddy current sensor digital interface
	eddy_current_sensor_init();
	eddy_current_sensor_enable();

	// AMDS
	gpio_mux_set_device(0, GPIO_MUX_DEVICE2);


}

#endif //APP_CABINET
