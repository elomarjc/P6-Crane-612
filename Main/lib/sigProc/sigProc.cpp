#include "sigProc.h"

/*
First-order low-pass filter with time constant tau. Time unit is seconds.
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

void low_pass::restart(double value){
    output_val = value;
}

lead_lag::lead_lag(float a, float b, float k){
    xa = a;
    xb = b;
    xk = k;
}

double lead_lag::update(double input){
    uint32_t xtime = micros();
    double outsig = update(input,float(xtime-prev_time)/1e6);
    prev_time = xtime;
    return outsig;
}

double lead_lag::update(double input, float dtime){
    float output_val = (xk*(input*(1+xa*dtime)-prev_error)+prev_output)/(1+xb*(dtime));
    prev_output = output_val;
    prev_error = input;
    
    return output_val;
}

void lead_lag::restart(double value){
    prev_output = value;
    prev_error = 0;
}

/*
Basic PID-controller with low-pass support for D-controller.

*/
PID::PID(double Kp, double Ki, double Kd, float tau, bool ideal){
    if (ideal){
        xKp = Kp;
        xKi = Ki*Kp;
        xKd = Kd*Kp;  
    } else {
        xKp = Kp;
        xKi = Ki;
        xKd = Kd;     
    }
    if (tau > 0){
        lp = low_pass(tau);
        dlp = true;
    } else {
        dlp = false;
    }
}

double PID::update(double error){
    uint32_t xtime = micros();
    double outsig = update(error,xtime-prev_time);
    prev_time = xtime;

    return outsig;
}

double PID::update(double error,uint32_t dtime){
    double outsig = 0;

    if (xKp){ // proportional
        outsig += xKp*error;
    }

    if (xKd && dlp){ // derivative with lp
        differential = (xKd*1e6*(lp.update(error-prev_error, dtime/1e6)))/dtime;
        outsig += differential;
    } else if (xKd) { // derivative
        differential = (xKd*1e6*(error-prev_error))/dtime;
        outsig += differential;
    }
    
    if (xKi){ // integral
        integral += (xKi*error*dtime/1e6);      
        outsig += integral;
    }

    prev_error = error;

    return outsig;
}

void PID::restart(){
    lp.restart(0);
    prev_error = 0;
    integral = 0;
}


/*
Basic 3-parameter IIR-filter of the format:

b[0] + b[1]*z^-1 + b[2]*z^-2 
----------------------------
a[0] + a[1]*z^-1 + a[2]*z^-2

*/

IIR::IIR(){}

IIR::IIR(float a_in[3], float b_in[3]){

    a[0] = a_in[0]; a[1] = a_in[1]; a[2] = a_in[2];
    b[0] = b_in[0]; b[1] = b_in[1]; b[2] = b_in[2];

    x[0] = 0; x[1] = 0; x[2] = 0; 
    y[0] = 0; y[1] = 0; y[2] = 0;

}

float IIR::update(float input){
    x[2] = x[1]; x[1] = x[0];
    x[0] = input;

    y[2] = y[1]; y[1] = y[0];
    
    float output = 
        + x[0]*b[0]
        + x[1]*b[1]
        + x[2]*b[2]

        - y[1]*a[1]
        - y[2]*a[2] ;

    output = output/a[0];

    y[0] = output;

    return output;
}

void IIR::updateParams(float a_in[3], float b_in[3]){
   *a = *a_in;
   *b = *b_in;
}

NotchFilter::NotchFilter() {}

NotchFilter::NotchFilter(float Fc, float Fb,float Ts) {

    xFc = Fc; xFb = Fb; xTs = Ts;
    frac x = calcParams(Fc,Fb,Ts);
    iir = IIR(x.a,x.b);

}

float NotchFilter::update(float input){
    return iir.update(input);
}

void NotchFilter::updateFrequency(float Fc){
    xFc = Fc;
    frac x = calcParams(xFc,xFb,xTs);
    iir.updateParams(x.a,x.b);
}

void NotchFilter::updateBandwidth(float Fb){
    xFb = Fb;
    frac x = calcParams(xFc,xFb,xTs);
    iir.updateParams(x.a,x.b);
}

frac NotchFilter::calcParams(float Fc, float Fb, float Ts){

    // Pre-warp frequency  
    float wc = 2/Ts*tan( Fc*2*PI *Ts/2);
    float wb = Fb*2*PI;

    frac x;

    // Calculate parameters
    float b_temp = wc*wc*Ts*Ts;
    x.b[0] =  b_temp + 4;
    x.b[1] = (b_temp - 4)*2;
    x.b[2] =  b_temp + 4; 

    x.a[0] = x.b[0] + 2*wb*Ts;
    x.a[1] = x.b[1];
    x.a[2] = x.b[2] - 2*wb*Ts;

    // Serial.print(x.b[0]/x.a[0]);Serial.print(", ");
    // Serial.print(x.b[1]/x.a[0]);Serial.print(", ");
    // Serial.println(x.b[2]/x.a[0]);

    // Serial.print(x.a[0]/x.a[0]);Serial.print(", ");
    // Serial.print(x.a[1]/x.a[0]);Serial.print(", ");
    // Serial.println(x.a[2]/x.a[0]);

    return x;
}

forwardEuler::forwardEuler(){}

float forwardEuler::update(float input){
    output = (input - prev_input) / (micros() - prev_time);
    prev_time = micros();
    prev_input = input;
    return output*1e6;
}

// EnableFix function not in use
enableFix::enableFix(){}

bool enableFix::update(float actualCurrent, int demantedCurrent, float difference, uint32_t timeMillis){
    enable = 1;
    // Converte pwm value to current 
    currentDemanted = ((5*demantedCurrent)/51)-(25/2);

    // Converte analog current to actual current and check for correct values
    if(actualCurrent>819.2){
        actualCurrent = 819.2;
    }

    currentAcutual = ((actualCurrent*0.0048828125)-2)*5;


    // Calculate current difference
    diffCurrent = currentDemanted - currentAcutual;

    // if difference is too big disable xx time
    if (abs(diffCurrent)>8){
        prev_time = prev_time+1;

        if (prev_time<=15){
            enable = 0;
        }
        if (prev_time>=30){
            enable = 1;
            prev_time = 0;

        }
    }
    return enable;
}