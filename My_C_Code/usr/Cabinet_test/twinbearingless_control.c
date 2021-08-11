
#include "usr/Cabinet_test/twinbearingless_control.h"
#include "usr/Cabinet_test/cabinet.h"
#include "usr/Cabinet_test/analog_sensor.h"
#include "usr/Cabinet_test/mb_sensor.h"
#include "usr/Cabinet_test/controllers.h"


#define INV1 5
#define INV2 2
#define INV3 3

twinbearingless_control twin_control_data;
PI_discrete_para PI_inv1;
PI_discrete_para PI_inv2;
PI_discrete_para PI_inv3;


twinbearingless_control *init_twinbearingless(void){

    twin_control_data.is_init = 1;
    twin_control_data.twin_inv1.Num_inv = INV1;
    twin_control_data.twin_inv2.Num_inv = INV2;
    twin_control_data.twin_inv3.Num_inv = INV3;
    twin_control_data.twin_inv1.inv = get_three_phase_inverter(twin_control_data.twin_inv1.Num_inv);
    twin_control_data.twin_inv2.inv = get_three_phase_inverter(twin_control_data.twin_inv2.Num_inv);
    twin_control_data.twin_inv3.inv = get_three_phase_inverter(twin_control_data.twin_inv3.Num_inv);
    twin_control_data.twin_inv1.PI = 
    twin_control_data.twin_inv2.PI = 
    twin_control_data.twin_inv3.PI = 
    return &twin_control_data;
};

void update_mes_current(twin_data *data){
    data->Idq0_m1[0] = data->Idq0[0];
    data->Idq0_m1[1] = data->Idq0[1];
    data->Idq0_m1[2] = data->Idq0[2];

    if (inv->HW->sensor.enable){
        get_currents_three_phase_abc(data->Iabc, data->inv);
    }else{
        get_mb_currents_three_phase_abc(data->Iabc, data->inv);
    }

    abc_to_dq0(data->Iabc, data->Idq0, data->theta_rad);

}
