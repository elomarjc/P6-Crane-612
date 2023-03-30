#include"iir.h"

/*
Basic 3-parameter IIR-filter of the format:

b[0] + b[1]*z^-1 + b[2]*z^-2 
----------------------------
a[0] + a[1]*z^-1 + a[2]*z^-2

*/
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