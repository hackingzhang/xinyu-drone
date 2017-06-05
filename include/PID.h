#ifndef PID_H
#define PID_H


class PID
{
    public:
        PID();

        double calculate(double);

        double getKp();
        double getKi();
        double getKd();

        double getIntegral();

        virtual ~PID();
    protected:
    private:
        double Kp;
        double Ki;
        double Kd;
        double i;
        double d;
        double integral;
        double output_limit;
        double last_error;
        double output;

        double last_time;

};

#endif // PID_H
