#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
    Q_accel = 0.001;
    Q_bias = 0.003;
    R = 0.03;

    bias = 0;
    P[0][0] = 1;
    P[0][1] = 0;
    P[1][0] = 0;
    P[1][1] = 1;
}

void KalmanFilter::setAngle(double new_angle)
{
    angle = new_angle;
}

double KalmanFilter::updateAngle(double new_angle, double new_rate, double dt)
{
    rate = new_rate - bias;
    angle += dt * rate;

    // Error covariance estimation
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_accel);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Update Kalman gain (K = PH^T(HPH^T + R)^-1)
    S = P[0][0] + R;
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Update current state (angle, bias)
    y = new_angle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    // Update error covariance
    P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];

    return angle;
}

double KalmanFilter::getRate()
{
    return rate;
}

void KalmanFilter::setQaccel(double new_Qaccel)
{
    Q_accel = new_Qaccel;
}

void KalmanFilter::setQbias(double new_Qbias)
{
    Q_bias = new_Qbias;
}

void KalmanFilter::setR(double new_R)
{
    R = new_R;
}

KalmanFilter::~KalmanFilter()
{
    //dtor
}
