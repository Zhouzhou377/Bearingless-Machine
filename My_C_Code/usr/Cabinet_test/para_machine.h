#ifndef MACHINE_H
#define MACHINE_H


typedef struct para_machine_h {

    double R;
    double L_lk;
    double L_lm;
    double L_m;
    double L_m12;

} para_machine_h;

typedef struct para_twinmachine_control_single {

    double R;
    double L;

} para_twinmachine_control_single;


typedef struct para_twinmachine_control {

    para_twinmachine_control_single para_tq;
    para_twinmachine_control_single para_s1;
    para_twinmachine_control_single para_s2;

} para_twinmachine_control;


para_twinmachine_control *init_para_twinmachine_control(para_machine_h *para_machine_data);

#endif // MACHINE_H
