#include <Arduino.h>

#include "pinDefinitions.h"

//// VARIABLES ////
float angleOffset = 0.64;       // [deg]
unsigned long sampletime = 10;  // [ms]
// Define these based on values given in positionalValues() in Simple_gantry_code.ino
int minX = 865;  // left
int maxX = 53;   // right
int minY = 0;    // ceiling
int maxY = 873;  // floor

double Setpoint_y, Input_y, Output_y,
    Setpoint_x, Input_x, Output_x,
    Setpoint_theta, Input_theta, Output_theta;

// calculated K-values
// double Kp_y = 32.4, Ki_y = 0, Kd_y = 12.96;
double Kp_x = 1.59, Ki_x = 0, Kd_x = 1.15;
// double Kp_theta = 9, Ki_theta = 0, Kd_theta = 4.5;

// experimental K-values
double Kp_y = 1.5, Ki_y = 0, Kd_y = 12.96;
// double Kp_x = 1.59, Ki_x = 0, Kd_x = 2.3;
double Kp_theta = 0.9, Ki_theta = 0, Kd_theta = 0.45;

// double minCurrenty_up = -3.4, minCurrenty_down = 0.74,  // [A]
//     minCurrentx_left = 2, minCurrentx_right = -2;

double minCurrenty_up = -3.4, minCurrenty_down = 0.8,  // [A]
    minCurrentx_left = 2, minCurrentx_right = -2;

// double minPWMy_up = 0.33, minPWMy_down = 0.54,
//       minPWMx_left = 0.6, minPWMx_right = 0.4;

double currentLimity_up = -7.96, currentLimity_down = 8,  // [A]
    currentLimitx_left = 7.96, currentLimitx_right = -8;

//// GENERAL FUNCTIONS ////
// // Function that reads the inputs to the system and makes convertions
// void readInput() {
//   Input_y = (double)map(analogRead(pin_pos_y), minY, maxY, 0, 133) / 100;
//   Input_x = (double) map(analogRead(pin_pos_x), minX, maxX, 0, 400) /100;
//   Input_theta = getAngleFromHead();
// }

void dropload() {
  if (Input_y > (1.2 - 0.05)) {
    Serial.println("Releasing load");
    Serial3.println("M0");  // turn off the magnet
  } else {
    // Serial.println("Load is too high from the ground. Cannot release");
  }
}

void collectload() {
  Setpoint_y = 1.22;
  if (Input_y > (1.22 - 0.01)) {
    Serial.println("Collecting load");
    Serial3.println("M1");  // turn on the magnet
  } else {
    // Serial.println("Crane head is too high from the ground. Cannot collect");
  }
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

float getAngleFromHead() {
  float angle;
  if (Serial3.available() > 0) {
    String angleData = Serial3.readStringUntil('\n');
    angle = angleData.toFloat() - angleOffset;  // [deg]
    angle = angle * 3.141592 / 180;             // converting angle from [deg] to [rad]
    // Serial.println(angle);
  }
  return angle;
}