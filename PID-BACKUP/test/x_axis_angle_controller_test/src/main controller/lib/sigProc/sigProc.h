#include <Arduino.h>

class low_pass{
    public:
        low_pass(float tau);
        double update(double input);
        double update(double input,float dtime);
        void restart(double value);
    private:
        float xTau;
        uint32_t prev_time;
        double output_val;
};

class lead_lag{
    public:
        lead_lag(float a = 0, float b = 0, float k = 0);
        double update(double input);
        double update(double input,float dtime);
        void restart(double value = 0);
    private:
        float xa,xb,xk;
        float prev_error, prev_output;
        uint32_t prev_time;
};

class PID{
    public:
        PID(double Kp = 0 , double Ki = 0 , double Kd = 0 , float tau = 0, bool ideal = false);
        double update(double error);
        double update(double error,uint32_t dtime);
        void restart();
    private:
        double xKp, xKi, xKd;
        low_pass lp = low_pass(0);
        boolean dlp;
        uint32_t prev_time;
        double prev_error = 0;
        double differential = 0;
        double integral = 0;
};

class IIR{
    public:
        IIR();
        IIR(float a_in[3], float b_in[3]);
        float update(float input);
        float update(float input,uint32_t dtime);
        void restart();
    private:
        float a[3], b[3];
        float x[3], y[3];
        uint32_t prev_time;
        float prev_error = 0;
};

class NotchFilter{
    public:
        NotchFilter(float Fc, float Fb,float Ts);
        float update(float input);
        float update(float input,uint32_t dtime);
        void restart();
    private:
        IIR iir;
};

class forwardEuler{
    public:
        forwardEuler();
        float update(float input);
    private:
        uint32_t prev_time=100000000;
        float prev_input;
        float output;
};

// Not in use
class enableFix{
    public:
        enableFix();
        bool update(float actualCurrent, int demantedCurrent, float difference, uint32_t timeMillis);
    private:
        float currentAcutual = 0;
        float currentDemanted = 0;
        float diffCurrent = 0;
        bool enable = 0;
        uint32_t prev_time=100000000;
};
