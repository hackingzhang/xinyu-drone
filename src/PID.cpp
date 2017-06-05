#include "PID.h"

PID::PID()
{
    //ctor
    Kp = 1;
    Ki = 0.005;
    Kd = 1;
    last_error = 0.0;
    integral = 0.0;
    output = 0.0;
}

double PID::calculate(double error){
    integral += error;
    output = (Kp * error) + (Ki * integral) + Kd * (error - last_error);
    last_error = error;
    return output;
}

double PID::getIntegral(){
    return integral;
}

PID::~PID()
{
    //dtor
}
