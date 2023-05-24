#include "sigProc.h"

/*
First-order low-pass filter with time constant tau [s]
*/
low_pass::low_pass(float tau){
    xTau = tau;
}

double low_pass::update(double input){
    uint32_t xtime = micros();
    double outsig = update(input,(xtime-prev_time)/1e6);
    prev_time = xtime;
    return outsig;
}

double low_pass::update(double input,float dtime){
    float a = dtime/(dtime+xTau);
    output_val = output_val*(1-a)+input*a;
    return output_val;
}


forwardEuler::forwardEuler(){}

float forwardEuler::update(float input){
    output = (input - prev_input) / (micros() - prev_time);
    prev_time = micros();
    prev_input = input;
    return output*1e6;
}
