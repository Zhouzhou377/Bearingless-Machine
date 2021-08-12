#ifndef PARA_MACHINE_H
#define PARA_MACHINE_H


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


extern para_twinmachine_control para_machine_control;
extern para_machine_h para_machine_data;

para_twinmachine_control *init_para_twinmachine_control(void);

#endif // PARA_MACHINE_H
