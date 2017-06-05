#ifndef KALMANFILTER_H
#define KALMANFILTER_H


class KalmanFilter
{
    public:
        KalmanFilter();

        /******
        * setAngle - set initial angle
        * Params:
        *   angle - double, initial angle(units: degree)
        ******/
        void setAngle(double new_angle);

        /******
        * updateAngle - mian function of kalman filter
        * Params:
        *   new_angle - new angle from accelerometer
        *   new_rate - new rate from gyroscope
        *   dt - time interval
        ******/
        double updateAngle(double new_angle, double new_rate, double dt);

        /******
        * getRate - return the unbiased rate
        ******/
        double getRate();

        void setQaccel(double new_Qaccel);
        void setQbias(double new_Qbias);
        void setR(double new_R);

        virtual ~KalmanFilter();
    protected:
    private:
        double Q_accel; // Input noise covariance of accelerometer
        double Q_bias;  // Input noise covariance of gyroscope
        double R;   // Measurement noise covariance

        double angle;   // Angle calculated by kalman filter
        double bias;    // Gyroscope bias (zero drift)
        double rate;    // Unbiased rate calculated by kalman filter

        double P[2][2];  // Error covariance matrix
        double K[2];    // Kalman gain matrix
        double y;   // angle difference
        double S;   // Estimate error
};

#endif // KALMANFILTER_H
