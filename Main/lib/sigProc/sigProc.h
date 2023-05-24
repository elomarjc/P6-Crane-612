#include <Arduino.h>

class low_pass{
    public:
        low_pass(float tau);
        double update(double input);
        double update(double input,float dtime);
    private:
        float xTau;
        uint32_t prev_time;
        double output_val;
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
