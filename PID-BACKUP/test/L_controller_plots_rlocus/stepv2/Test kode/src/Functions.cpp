#include <Arduino.h>

bool joystickDeadZone(float dataJoystick){
    bool toReturn;
    float buffer = 5;
    
    if(511.5+buffer > dataJoystick  && dataJoystick > 511.5-buffer){
        toReturn = 0;
    }
    else{
        toReturn = 1;
    }
    return toReturn;
}

int joystickOutputFormat(float dataJoystick){
    int toReturn = int(0.19844*dataJoystick+26);
    return toReturn;
}

// Software endstops (works on the pwm value)
int endstop(int pwm, float min, float max, float pos){
    int pwmEndstop = 127;
    bool dir = 0;
    pwmEndstop = pwm;

    // Check direction (used so it can move in the other direction)
    if (pwm < 127) {
        dir = 0;
    }
    else {
        dir = 1;
    }
    
    // Chek if endstop is hit
    if (pos > max && dir == 1) {
    pwmEndstop = 127;
    }
    if (pos < min && dir == 0) {
    pwmEndstop = 127;
    }

    return pwmEndstop;
}

uint8_t currentToPwm(float current,float fric_dead, float cust_dead) {

    // Linear current -> pwm conversion
    float pwm = 10.2*current+127.5;

    // Add and compensate for deadzone (friction)
    if (abs(current) < cust_dead){
        pwm = 127.5;
    } else if (current < 0) {
        pwm = pwm - fric_dead;
    } else if (current > 0) {
        pwm = pwm + fric_dead;
    }

    // Output limiter to 10% and 90%
    if ((uint8_t)pwm > 255*0.9) return (uint8_t)255*0.9;
    else if ((uint8_t)pwm < 255*0.1) return (uint8_t)255*0.1;
    else return (uint8_t)pwm;
  
}

int pwmLinY(float pwm) {
    float tempPWM = pwm - 3.384;
    
    if(tempPWM<255*0.1){
        tempPWM = 255*0.1;
    }

    if(tempPWM>255*0.9){
        tempPWM = 255*0.9;
    }

    return(tempPWM);
}