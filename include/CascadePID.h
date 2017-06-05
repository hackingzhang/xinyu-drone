#ifndef CASCADEPID_H
#define CASCADEPID_H


class CascadePID
{
    public:
        CascadePID();
        double calculate(double, double);
        virtual ~CascadePID();
    protected:
    private:
        double outer_integral_max;
        double outer_integral_min;

        double inner_integral_max;
        double inner_integral_min;

        // outer PID params
        double outer_kp,
                outer_ki,
                outer_kd;
        double outer_integral;
        // outer output
        double outer_out;

        // inner PID params
        double inner_kp,
                inner_ki,
                inner_kd;
        double inner_integral;
        // inner output
        double inner_error,
                inner_error_last;

        // output
        double output;
};

#endif // CASCADEPID_H
