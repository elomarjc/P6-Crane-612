#include <Arduino.h>

#include "pinDefinitions.h"

//// VARIABLES ////
float angleOffset = -0.47;          // [deg]
unsigned long sampletimeXY = 10;    // [ms]
unsigned long sampletimeTHETA = 1;  // [ms]
// Define these based on values given in positionalValues() in Simple_gantry_code.ino
int minX = 855;  // left
int maxX = 44;  // right
int minY = 50;    // ceiling
int maxY = 873;  // floor

double Setpoint_y, Input_y, Output_y,
    Setpoint_x, Input_x, Output_x,
    Setpoint_theta, Input_theta, Output_theta;

double Kp_y = 32.4, Ki_y = 0, Kd_y = 12.96,
       Kp_x = 1, Ki_x = 3, Kd_x = 5,
       // Kp_x = 2.4, Ki_x = 0, Kd_x = 1.92
    Kp_theta = 22.6, Ki_theta = 0, Kd_theta = 11.3;

//// GENERAL FUNCTIONS ////
float getAngleFromHead() {
  float angle;
  if (Serial3.available() > 0) {
    String angleData = Serial3.readStringUntil('\n');
    angle = angleData.toFloat() - angleOffset;
    // Serial.println(angle);
  }
  return angle;
}

// Function that reads the inputs to the system and makes convertions
void readInput() {
  Input_y = analogRead(pin_pos_y);
  Input_x = analogRead(pin_pos_x);
  Input_theta = getAngleFromHead();
}


//// FOR Y-AXIS ////
void newSetpoint_y(double newSetpoint) {
  Setpoint_y = (double)map(newSetpoint * 100, 0, 133, minY, maxY);
}


//// FOR X-AXIS ////
void newSetpoint_x(double newSetpoint) {
  Setpoint_x = (double)map(newSetpoint * 100, 0, 400, minX, maxX);
}


//// FOR ANGLE ////
void angleCorrection() {
  Serial.println("--- Correcting angle ---");
  float angleSum = 0;
  int n = 1000;
  int i = 0;
  while (i < n) {
    if (Serial3.available() > 0) {
      String angleData = Serial3.readStringUntil('\n');
      angleSum += angleData.toFloat();
      i++;
    }
  }
  angleOffset = angleSum / n;
  Serial.print("Angle offset: ");
  Serial.println(angleOffset);
}
