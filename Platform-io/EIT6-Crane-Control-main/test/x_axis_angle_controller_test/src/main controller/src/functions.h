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


#endif