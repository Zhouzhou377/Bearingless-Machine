#ifndef CONTROL_STRUCTURE_H
#define CONTROL_STRUCTURE_H

typedef struct para_PI_discrete_normal{

    //Controller Parameters Calculated from Bilinear Transform
    double Ts;
    double Kp;
    double Ki;
    double state_1[3];

} para_PI_discrete_normal;

typedef struct para_lpf{

    //Controller Parameters Calculated from Bilinear Transform
    double Ts;
    double fs;

    double state_1[3];

} para_lpf;

typedef struct para_observer{
    double enable;
    double Ts;
    double Kd;
    double Kj;
    double state1;
    double state2;
    para_PI_discrete_normal para_PI;

} para_observer;

typedef struct para_anti_windup{

    double sat_low;
    double sat_high;
    double k;

} para_anti_windup;

typedef struct para_levitation_control{
    int enable;
    para_PI_discrete_normal para_PI;
    double Ts;
    double ka;
    double ba;
    double state_1[3];
    para_anti_windup para_anti_wp;
    para_lpf para_lpf;
    para_lpf para_delta_lpf;

} para_levitation_control;

typedef struct para_velocity_control{
    double wrm_max;
    para_lpf para_lpf;
    para_PI_discrete_normal para_PI;


} para_velocity_control;


void reset_states_3phase (double *state);
double get_encoder_pos(void);
void get_pos_w_mes(double theta_now, double *theta_pre, double *w);
void func_lpf(double *in, double *out, para_lpf *para_lpf, double *state);
void func_PI_normal(double *in, double *out, int Num_variable, para_PI_discrete_normal *para_PI, double *out_antiwp);
void func_observer_theta(double theta_mes, double *theta_est, double *w_est, double *w_est_hf, para_observer *para_ob, double tq);
void func_anti_windup(para_anti_windup *para_antiwp, double in, double *in_antiwp, double *out);
int protection_overcurrent(double iabc[3], double i_max);
#endif // SYS_PARAMETER_H
