#include<stdio.h>
#include<stdlib.h>
#include"iir.h"

float b[3] = {1.0f, -1.9553f, 0.9738f};
float a[3] = {1.0f, -1.7276f, 0.7462f};

float IMPULSE[100] = {1}; 

IIR filter1 = IIR(a,b);
IIR filter2 = IIR(a,b);

int main() {
    printf("\nstp = [");
    for (int i = 0; i < 100 ; i++) {
        printf("%f ",filter1.update(1));
    } printf("];");

    printf("\nimp = [");
    for (int i = 0; i < 100 ; i++) {
        printf("%f ",filter2.update(IMPULSE[i]));
    } printf("];\nt=1:100\nplot(t,stp,t,imp);\n");
}