#include <Arduino.h>

#include "pinDefinitions.h"

//// VARIABLES ////
float angleOffset = -1.33;      // [deg]
unsigned long sampletime = 10;  // [ms]
// Define these based on values given in positionalValues() in Simple_gantry_code.ino
int minX = 865;  // left
int maxX = 53;   // right
int minY = 0;    // ceiling
int maxY = 873;  // floor

// New global variables  FOR PATHING
int step = 0;
uint16_t failTime = 0;
float xContainer;
bool magnet_sw = 0;

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

double minCurrenty_up = -3.4, minCurrenty_down = 0.8,  // Minimum [A] needed to overcome static friction
    minCurrentx_left = 2, minCurrentx_right = -2;

// double minPWMy_up = 0.33, minPWMy_down = 0.54, // Minimum duty cycle needed to overcome static friction
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


//// FOR ANGLE ////
void angleCorrection() {
  Serial.println("--- Correcting angle ---");
  float angleSum = 0;
  int n = 20;
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
    angle = angle * PI / 180;                   // converting angle from [deg] to [rad]

    if (abs(angle) < 0.0174532925) { // ignore angles smaller than 1 deg
      angle = 0;
    }
    // Serial.println(angle);
  }
  return angle;
}

// Make sure to replace
// xPos = x position of trolley
// yPos = y position of trolley
// angle = angle input

//// FOR PATHING ////
int pathAtoB(float xPos, float yPos, float xContainer) {
  // Move to above qauy
  if (step == 0) {
    Setpoint_x = 0.5;
    Setpoint_y = 0.7;
    if (0.40 > xPos || xPos > 0.60 || 0.40 > xContainer || xContainer > 0.60) {  // If trolley is not above container
      failTime = millis();
      // Serial.println("Trolley is not above container.");
    } else if (millis() > failTime + 2000)  // 2000 or 400
    {                                       // If head has been above container for 2 seconds
      // Serial.println("Trolley is above container.");
      step = 1;
    }
  }

  // Lower head onto container
  if (step == 1) {
    // Serial.println("Step = 2, lower head onto container");
    Setpoint_y = 1.20;
    if (yPos < 1.20) {
      failTime = millis();
    } else if (millis() > failTime + 400) {
      Serial3.println("M1");
      magnet_sw = 1;
      step = 2;
    }
  }

  // Hoist contrainer
  if (step == 2) {
    // Serial.println("Step = 3, move to safety point");
    Setpoint_y = 0.7;
    if (yPos == 0.7) {
      step = 3;
      // Serial.println("//Step3 passed");
    }
  }

  // Move above ship
  if (step == 3) {
    // Serial.println("Step = 4, move above ship");
    Setpoint_x = 2.95;
    if (2.90 > xPos || xPos > 3) {  // if not within position
      failTime = millis();
    } else if (millis() > failTime + 6000) {  // wait 6 s before going down
      step = 4;
    }
  }

  // Move down to ship and turn off electro magnet.
  if (step == 4) {
    // Serial.println("Step = 5, move downto ship and turn off electro magnet.");
    Setpoint_y = 1.30;
    if (yPos == 1.20) {
      Serial3.println("M0");  // Drop load
      magnet_sw = 0;
      Setpoint_y = 0.7;  // Go back up
      step = 5;
      // Serial.println("//PATH A TO B DONE!");
    }
  }

  return step;
}

int pathBtoA(float xPos, float yPos, float xContainer) {
  // Move to above shipment
  if (step == 0) {
    Serial.println("Step = 0");
    Setpoint_x = 3;
    Setpoint_y = 0.7;
    if (2.90 > xContainer || xContainer > 3.10 || 2.90 > xPos || xPos > 3.10) {  // if not within position
      Serial.println("Trolley not in start position");
      failTime = millis();
    } else if (millis() > failTime + 6000) {  // wait 6 s before going down
      step = 1;
    }
  }

  // Lower head onto container
  if (step == 1) {
    Serial.println("Step = 1, lower head onto container");
    Setpoint_y = 1.20;
    if (yPos < 1.20) {
      failTime = millis();
    } else if (millis() > failTime + 400) {
      Serial3.println("M1");
      magnet_sw = 1;
      step = 2;
    }
  }

  // Hoist contrainer
  if (step == 2) {
    Serial.println("Step = 2, move to safety point");
    Setpoint_y = 0.7;
    if (yPos == 0.7) {
      step = 3;
      Serial.println("//Step3 passed");
    }
  }

  // Move above quay
  if (step == 3) {
    Serial.println("Step = 4, move above ship");
    Setpoint_x = 0.5;
    if (0.40 > xPos || xPos > 0.60 || 0.40 > xContainer || xContainer > 0.60) {  // If trolley is not above container. pm 2 cm
      failTime = millis();
      // Serial.println("Trolley is not above container.");
    } else if (millis() > failTime + 5000) {  // If head has been above container for 5s
      Serial.println("Trolley is above container.");
      step = 4;
    }
  }

  // Move down to quay and turn off electro magnet.
  if (step == 4) {
    Serial.println("Step = 4, move downto ship and turn off electro magnet.");
    Setpoint_y = 1.30;
    if (yPos == 1.20) {
      Serial3.println("M0");  // Drop load
      magnet_sw = 0;
      Setpoint_y = 0.7;  // Go back up
      step = 5;
      Serial.println("//PATH B TO A DONE!");
    }
  }

  return step;
}


//// OLD FUNCTIONS ////
// void dropload() {
//   if (Input_y > (1.2 - 0.05))  // if position of container is less than 5 cm from ground
//   {
//     Serial.println("Releasing load");
//     Serial3.println("M0");  // turn off the magnet
//     magnet_sw = 0;
//   } else {
//     // Serial.println("Load is too high from the ground. Cannot release");
//   }
// }

// void collectload() {
//   Setpoint_y = 1.22;
//   if (Input_y > (1.22 - 0.01)) {
//     Serial.println("Collecting load");
//     Serial3.println("M1");  // turn on the magnet
//     magnet_sw = 1;
//   } else {
//     // Serial.println("Crane head is too high from the ground. Cannot collect");
//   }
// }