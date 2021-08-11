#include "usr/Cabinet_test/controllers.h"
//#include "usr/Cabinet_test/twinbearingless_control.h"


PI_discrete_para *init_PIregulator(void){

    //populate the PI container based on the bilinear transformation to create difference equations
    container->Ts = Ts;
    container->kp = kp;
    container->kei = ki*(Ts/2.0);
    container->r_star = 0;
    container->error_old = 0;
    container->clamp = 0;
    container->mn = 0;
    container->mn_i_old = 0;

}

void populate_PID_Container(double kp, double ki, double kd, double wp, double Ts, PID_Container *container){

    //populate the PID container based on the bilinear transformation to create difference equations
    container->Ts = Ts;
    container->kp = kp;
    container->kei = ki*(Ts/2.0);
    container->ked = (2.0*wp*kd)/(Ts*wp + 2.0);
    container->kmd = (2.0 - Ts*wp)/(Ts*wp + 2.0);
    container->r_star = 0;
    container->error_old = 0;
    container->clamp = 0;
    container->mn = 0;
    container->mn_i_old = 0;
    container->mn_d_old = 0;

}

void reset_PI_controller(PI_Container *container){

    container->error_old = 0;
    container->mn = 0;
    container->mn_i_old = 0;
    container->clamp = 0;
    container->r_star = 0;
}

void reset_PID_controller(PID_Container *container){

    container->error_old = 0;
    container->mn = 0;
    container->mn_i_old = 0;
    container->mn_d_old = 0;
    container->clamp = 0;
    container->r_star = 0;
}


void populate_LowPass_Container(double Ts, double tau, LowPass_Container *container){

    //populate the LowPass container based on the bilinear transformation to create difference equations
    container->Ts = Ts;
    container->kxLP = Ts/(Ts + 2*tau);
    container->kyLP = (2*tau - Ts)/(Ts + 2*tau);
    container->xn_1 = 0;
    container->yn_1 = 0;

}

double PI_update(PI_Container *container, double r_measured){

    //compute errors
    double en_1 = container->error_old;
    double en = container->r_star - r_measured;

    //compute proportional manipulation
    double mn_p = container->kp*en;

    //compute integral manipulation
    double mn_i = container->mn_i_old + container->kei*(en + en_1)*(1 - container->clamp);

    //total manipulation
    double mn = mn_p + mn_i;

    //set values in controller container
    container->mn = mn;
    container->mn_i_old = mn_i;
    container->error_old = en; //set old error to new error

    return mn;
}

double PID_update(PID_Container *container, double r_measured){

    //compute errors
    double en_1 = container->error_old;
    double en = container->r_star - r_measured;

    //compute proportional manipulation
    double mn_p = container->kp*en;

    //compute integral manipulation
    double mn_i = container->mn_i_old + container->kei*(en + en_1)*(1 - container->clamp);

    //compute derivative manipulation
    double mn_d = container->kmd*container->mn_d_old + container->ked*(en - en_1);

    //total manipulation
    double mn = mn_p + mn_i + mn_d;

    //set values in controller container
    container->mn = mn;
    container->mn_i_old = mn_i;
    container->mn_d_old = mn_d;
    container->error_old = en; //set old error to new error

    return mn;
}

double LowPass_update(LowPass_Container *container, double xn){

    //compute new filter output
    double yn_1 = container->yn_1;
    double xn_1 = container->xn_1;
    double yn = container->kyLP*yn_1 + container->kxLP*(xn + xn_1);

    //set container quantities
    container->xn_1 = xn;
    container->yn_1 = yn;

    //return filter output
    return yn;
}
