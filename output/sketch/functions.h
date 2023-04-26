#line 1 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\Simple_gantry_code\\functions.h"
#ifndef manuelFunctions_h
#define manuelFunctions_h

#include <Arduino.h>
#include "dataStructures.h"

bool joystickDeadZone(float dataJoystick);
int endstop(int pwm, float min, float max, float pos);
uint8_t currentToPwmY(double current, float ySpeed, bool magnetSw);
uint8_t currentToPwmX(double current, float xSpeed, bool* enableXMotor);
void turnOnElectromagnet(bool status, int LEDPin);
void getSerialReference(HardwareSerial *serial,xy_float *reference);
void getAngleSensor(HardwareSerial *serial, float *angle);

class fastReader{
    public:
        fastReader(HardwareSerial *serial);
        bool getFloatln(float *output);
    private:
        HardwareSerial *intSerial;
        String buffer;
};

#endif