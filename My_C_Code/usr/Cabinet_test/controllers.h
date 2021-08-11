#ifndef CONTROLLERS_H
#define CONTROLLERS_H

typedef struct PI_discrete_para{

    //Controller Parameters Calculated from Bilinear Transform
    double Ts;
    double tau_p;
    double Ap;
    double Bp;
    double wd;
    double tau_d;
    double Ad;
    double Bd;

} PI_discrete_para;

/*
typedef struct PID_Container{

    //Controller Parameters Calculated from Bilinear Transform
    double Ts;
    double kp;
    double kei;
    double ked;
    double kmd;

    //Commanded Value
    double r_star;

    //store previous error and old manipulations
    double error_old;
    double mn_i_old;
    double mn_d_old;

    //store current manipulation
    double mn;

    //clamping status
    int clamp;

} PID_Container;*/

typedef struct LowPass_Container{

    //Filter Parameters Calculated from Bilinear Transform
    double Ts;
    double kxLP;
    double kyLP;

    double xn_1;
    double yn_1;


} LowPass_Container;

void populate_PI_Container(double kp, double ki, double Ts, PI_Container *container);
void populate_PID_Container(double kp, double ki, double kd, double wp, double Ts, PID_Container *container);
void populate_LowPass_Container(double Ts, double tau, LowPass_Container *container);

void reset_PI_controller(PI_Container *container);
void reset_PID_controller(PID_Container *container);

double PI_update(PI_Container *container, double r_measured);
double PID_update(PID_Container *container, double r_measured);
double LowPass_update(LowPass_Container *container, double input);

#endif // CONTROLLERS_H
