#include<stdio.h>
#include<stdlib.h>

using namespace std;

class IIR{
    public:
        IIR(float a_in[3], float b_in[3]);
        float update(float input);
        float update(float input,u_int32_t dtime);
        void restart();
    private:
        float a[3], b[3];
        float x[3], y[3];
        u_int32_t prev_time;
        float prev_error = 0;
};
