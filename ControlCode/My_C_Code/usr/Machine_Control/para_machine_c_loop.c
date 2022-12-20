#include "usr/Machine_Control/sys_parameter.h"
#include "usr/Machine_Control/para_machine_c_loop.h"
#include "usr/Machine_Control/BIM/bim_para_machine.h"
#include "usr/Machine_Control/BP3/bp3_para_machine.h"
para_c_loop_machine_control para_machine_control;
para_machine_h para_machine_data;

#define PARA_R_S (1.0)
#define PARA_L_LK (0.00063)
#define PARA_L_LM (0.0)
#define PARA_L_M (0.00265)
#define PARA_L_M12 (0.0)

para_c_loop_machine_control *init_para_c_loop_machine_control(void){

    double L_l, L_m;
    if(!BM_ENABLE){
        para_machine_data.R = PARA_R_S;
        para_machine_data.L_lk = PARA_L_LK;
        para_machine_data.L_lm = PARA_L_LM;
        para_machine_data.L_m = PARA_L_M;
        para_machine_data.L_m12 = PARA_L_M12;

        // torque equivalent parameters
        para_machine_control.para_tq.R = para_machine_data.R/2;
        L_l = (para_machine_data.L_lk + para_machine_data.L_lm)*0.5;
        L_m = (para_machine_data.L_m + para_machine_data.L_m12)*3/4;
        para_machine_control.para_tq.L = L_l + L_m;

        // suspension 1 equivalent parameters
        para_machine_control.para_s1.R = para_machine_data.R;
        L_l = (para_machine_data.L_lk - para_machine_data.L_lm);
        L_m = (para_machine_data.L_m - para_machine_data.L_m12)*3/2;
        para_machine_control.para_s1.L = L_l + L_m;

        // suspension 2 equivalent parameters
        para_machine_control.para_s2.R = para_machine_data.R;
        L_l = (para_machine_data.L_lk - para_machine_data.L_lm);
        L_m = (para_machine_data.L_m - para_machine_data.L_m12)*3/2;
        para_machine_control.para_s2.L = L_l + L_m;

        // torque equivalent parameters
        para_machine_control.para_tq2.R = para_machine_data.R/2;
        L_l = (para_machine_data.L_lk + para_machine_data.L_lm)*0.5;
        L_m = (para_machine_data.L_m + para_machine_data.L_m12)*3/4;
        para_machine_control.para_tq2.L = L_l + L_m;
    }else if(BIM_ENABLE){
        para_bim * para_bim;
        para_bim = get_para_bim();
        // torque equivalent parameters
        
        para_machine_control.para_tq.R = 0.52;
        para_machine_control.para_tq.L = 0.0058;

        // suspension 1 equivalent parameters
        para_machine_control.para_s1.R = 1.007;
       
        para_machine_control.para_s1.L = 0.0041;
        /*
        para_machine_control.para_tq.R = para_bim->para_machine.rs/2.0;
        para_machine_control.para_tq.L = para_bim->para_machine.Ls;

        // suspension 1 equivalent parameters
        para_machine_control.para_s1.R = para_bim->para_machine.rs*2.0;
        L_l = para_bim->para_machine.Lls*2.0;
        L_m = para_bim->para_machine.Lss*4.0;
        para_machine_control.para_s1.L = L_l + L_m;*/

    }else if(BP3_ENABLE){
        para_bp3 * para_bp3;
        para_bp3 = get_para_bp3();
        // torque equivalent parameters
        
        para_machine_control.para_tq.R = 0.52;
        para_machine_control.para_tq.L = 0.0058;

        // suspension 1 equivalent parameters
        para_machine_control.para_s1.R = 1.007;
       
        para_machine_control.para_s1.L = 0.0041;
    }

    return &para_machine_control;
}

