#ifndef manuelFunctions_h
#define manuelFunctions_h

#include <Arduino.h>

bool joystickDeadZone(float dataJoystick);
int joystickOutputFormat(float dataJoystick);
int endstop(int pwm, float min, float max, float pos);
uint8_t currentToPwm(float current, float fric_dead, float cust_dead);
int pwmLinY(float pwm);

#endif