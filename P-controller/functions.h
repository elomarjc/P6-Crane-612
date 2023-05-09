#ifndef manuelFunctions_h
#define manuelFunctions_h

#include <Arduino.h>

#include "pinDefinitions.h"

// define these based on values given in positionalValues() [cm]
int minX = 136;  // left
int maxX = 951;  // right
int minY = 50;   // ceiling
int maxY = 873;  // floor

float goodMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float currentPosX() {
  return (float)map(analogRead(pin_pos_x), minX, maxX, 0, 400) / 100;  // [cm]
}

// float currentPosY()
// {
//   return (float) map(analogRead(pin_pos_y), minY, maxY, 0, 133) / 100; // [m]
// }

float currentPosY() {
  return goodMap(analogRead(pin_pos_y), minY, maxY, 0, 1.33);  // [m]
}

float errorY(float refVal) {
  // return goodMap(refVal - currentPosY(), -1.33, 1.33, 0.1, 0.9);
  return refVal - currentPosY();
}

void setVelocityX(float velocity)  // [0.1; 0.9], <0.5 to the left, >0.5 to the right, =0.5 stand still, speed is determined as difference between 0.5 and given value
{
  analogWrite(pin_pwm_x, 255 * velocity);
}

void setVelocityY(float velocity)  // [0.1; 0.9], <0.5 down, >0.5 up, =0.5 stand still, speed is determined as difference between 0.5 and given value
{
  analogWrite(pin_pwm_y, 255 * velocity);
}

void angleCorrection() {
  Serial.println("--- Correcting angle ---");
  int angleOffset = 0;
  float angleSum = 0;
  int n = 100;
  int i = 0;
  while (i < n) {
    if (Serial3.available() && Serial3.read() == '\n') {
      String angleData = Serial3.readStringUntil('\n');
      angleSum += angleData.toFloat();
      i++;
    }
  }
  angleOffset = angleSum / n;
  Serial.print("Angle offset: ");
  Serial.println(angleOffset);
}

#endif