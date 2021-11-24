#ifndef PARA_MACHINE_C_LOOP_H
#define PARA_MACHINE_C_LOOP_H

typedef struct para_machine_h {

    double R;
    double L_lk;
    double L_lm;
    double L_m;
    double L_m12;

} para_machine_h;

typedef struct para_c_loop_machine_control_single {

    double R;
    double L;

} para_c_loop_machine_control_single;


typedef struct para_c_loop_machine_control {

    para_c_loop_machine_control_single para_tq;
    para_c_loop_machine_control_single para_s1;
    para_c_loop_machine_control_single para_s2;
    para_c_loop_machine_control_single para_tq2;

} para_c_loop_machine_control;


extern para_c_loop_machine_control para_machine_control;
extern para_machine_h para_machine_data;

para_c_loop_machine_control *init_para_c_loop_machine_control(void);

#endif // PARA_MACHINE_H
