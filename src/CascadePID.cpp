#include "CascadePID.h"

CascadePID::CascadePID()
{
    outer_integral_max = 260;
    outer_integral_min = -260;

    inner_integral_max = 300;
    inner_integral_min = -300;

    outer_kp = 8;
    outer_ki = 0.0;
    outer_kd = 0.0;

    outer_integral = 0.0;

    inner_kp = 0.2;
    inner_ki = 0.0;
    inner_kd = 0.1;

    inner_integral = 0.0;

    inner_error = 0.0;
    inner_error_last = 0.0;

    outer_out = 0.0;
    output = 0.0;
}

double CascadePID::calculate(double error,double gyro){

/********************** Outer PID *********************/
    outer_integral += error;

    // anti integration saturation
    if(outer_integral > outer_integral_max){
        outer_integral = outer_integral_max;
    } else if(outer_integral < outer_integral_min){
        outer_integral = outer_integral_min;
    }

    outer_out = (outer_kp * error) + (outer_ki * outer_integral) + (outer_kd * gyro);

/********************** Inner PID *********************/
    inner_error = outer_out - gyro;
    inner_integral += inner_error;

    if(inner_integral > inner_integral_max){
        inner_integral = inner_integral_max;
    } else if(inner_integral < inner_integral_min){
        inner_integral = inner_integral_min;
    }

    output = (inner_kp * inner_error) +
             (inner_ki * inner_integral) +
             (inner_kd * (inner_error - inner_error_last));

    inner_error_last = inner_error;

    return output;
}

CascadePID::~CascadePID()
{
    //dtor
}
