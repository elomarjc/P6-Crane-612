#include <Arduino.h>
#include "functions.h"

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

// Software endstops
int endstop(int pwm, float min, float max, float pos){

    // Check if endstop is hit
    if ((pos > max && pwm > 127) || (pos < min && pwm < 127)) {
        Serial.println("Software endstop active");
        return 127;
    } else {
        return pwm;
    }
}

uint8_t currentToPwmY(double current, float ySpeed, bool magnetSw) {

    // Make more linear for y-axis (axis = 0)
    float coulombFriction = 0;
    // Adjust for gravity
    if (magnetSw == 1){
        current = current - 1.35; // Current gravity with container
        coulombFriction = 3.0;
    } else {
        current = current - 0.33; // Current gravity without container
        coulombFriction = 2.4;
    }
    
    // Adjust for friction
    if (abs(current) < 0){ //This number can be set to something larger than 0 if no movement is wanted for small currents
        current = 0;
    } else if (ySpeed < 0) {
        current = current - coulombFriction; // Columb friction current
    } else if (ySpeed > 0) {
        current = current + coulombFriction; // Columb friction current
    }

    // Set max speed
    if(abs(ySpeed)>0.5){
        current = 0; 
    }

    // Equalize currents around gravity
    if (current > 10 - 1.35 && magnetSw ==1) {
        current = 10 - 1.35;
    } else if (current > 10 - 0.33 && magnetSw ==0) {
        current = 10 - 0.33;
    }

    //Check for correct current value
    if (current > 10) {
        current = 10;
    } else if (current < -10) {
        current = -10;
    }
    
    // Linear current -> pwm conversion
    float pwm = 10.2*current+127.5;
    return (uint8_t)pwm;
}

uint8_t currentToPwmX(double current, float xSpeed, bool* enableXMotor){
        if (abs(current) < 0.3){ //This number can be set to something larger than 0 if no movement is wanted for small currents
            current = 0;
            *enableXMotor = false;
        } else if (xSpeed < 0) {
            current = current - 2.24; // Columb friction current
            *enableXMotor = true;
        } else if (xSpeed > 0) {
            current = current + 2.24; // Columb friction current
            *enableXMotor = true;
        }

        if(current > 10){
            current = 10;
        }
        if(current < -10){
            current = -10;
        }
    
        // Linear current -> pwm conversion
        float pwm = 10.2*current+127.5;
        return (uint8_t)pwm;
}

void turnOnElectromagnet(bool status, int LEDPin){
    if (status == true) {
    digitalWrite(LEDPin,HIGH);
    Serial3.println("M1");
  }

  if(status==false){
    digitalWrite(LEDPin,LOW);
    Serial3.println("M0");
  }
}

/*
Updates the reference x,y values from some serial input.

The format to send is the following:

x:X.X\n
and
y:Y.Y\n

Where X.X and Y.Y denote the reference point.
When using the Arduino Serial terminal, the \n is added autonatically.
*/
void getSerialReference(HardwareSerial *serial,xy_float *reference) {
    if ( serial->available() > 0 ) {
        String input = serial->readStringUntil(*"\n");

        // Get value of x and y from serial string
        int xIndex = input.indexOf("x:");
        int yIndex = input.indexOf("y:");

        if (xIndex > -1 && yIndex == -1){
            float value = input.substring(xIndex+2,input.length()).toFloat();
            if (value > 0 && value < 4) {reference->x = value;}
        }
        else if (yIndex > -1 && xIndex == -1){
            float value = input.substring(yIndex+2,input.indexOf(*"\n")).toFloat();
            if (value > 0 && value < 1.3) {reference->y = value;}
        }

        // Empty buffer
        while (serial->available() > 0) {serial->read();}
    }
}

// Fetch angle from serial buffer
void getAngleSensor(HardwareSerial *serial, float *angle){
    if (serial->available() > 0) {
        String angleData = serial->readStringUntil(*"\n");
        *angle = angleData.toFloat();
    }
}