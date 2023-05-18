#include <Arduino.h>

#include "pinDefinitions.h"

//// VARIABLES ////
float angleOffset = 0.63;       // [deg]
unsigned long sampletime = 10;  // [ms]
// Define these based on values given in positionalValues() in Simple_gantry_code.ino
int minX = 855;  // left
int maxX = 44;   // right
int minY = 0;    // ceiling
int maxY = 873;  // floor

//New global variables  FOR PATHING
int step = 0;
uint16_t failTime =0;
float xContainer, yContainer;

double Setpoint_y, Input_y, Output_y,
    Setpoint_x, Input_x, Output_x,
    Setpoint_theta, Input_theta, Output_theta;

// calculated K-values
double Kp_y = 32.4, Ki_y = 0, Kd_y = 12.96;
double Kp_x = 1.59, Ki_x = 0, Kd_x = 1.15;
double Kp_theta = 9, Ki_theta = 0, Kd_theta = 4.5;

// experimental K-values
// double Kp_y = 2, Ki_y = 0, Kd_y = 12.96;
// double Kp_x = 3, Ki_x = 0, Kd_x = 2.3;
// double Kp_theta = 0.9, Ki_theta = 0, Kd_theta = 0.45;

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

//Make sure to replace
// xPos = x position of trolley
// yPos = y position of trolley
// angle = angle input

//// FOR PATHING ////
int pathAtoB (float xPos, float yPos, float xContainer, float yContainer){
    if (step==0) {    
        Serial.println("Step = 0");
        //If at start position
        if (0.28<xPos && xPos<0.32 && 0.48<yPos && yPos<0.52){ //put the right values
            step=1;
            Serial.println("Trolley is at the start position");
        } else {
            Serial.println("Not in start position");
        }
        delay(2000);
    } 

    // Move to above qauy
    if (step==1) {
        Setpoint_x = 0.9;
        if (0.88>xPos || xPos>0.92) {    //If trolley is not above container. pm 2 cm
           failTime = millis();
           //Serial.println("Trolley is not above container.");
        } else if (millis() > failTime + 300) { //If head has been above container for 0.5s 
           Serial.println("Trolley is above container.");
           step = 2;
           Serial3.println("M1");   
           Serial.println("//Step1 passed");
       }
       delay(5000);
    }

    // Lower head onto container
    if (step==2) {
        Serial.println("Step = 2, lower head onto container");
        Setpoint_y = 1.15;
        if (yPos<1.15) {
            failTime = millis();
        } else if (millis() > failTime + 400) {
            step=4;
            Serial.println("Else if step=4");
        }
        delay(5000);
    }

    // Hoist contrainer
    if (step==3) {
        Serial.println("Step = 3, move to safety point");
        Setpoint_y = 0.9;
        if (yPos < 0.9) {        
            step=4;
            Serial.println("//Step3 passed");
        }
        delay(5000);
    }

    // Move above ship
    if (step==4) {
        Serial.println("Step = 4, move above ship");
        Setpoint_x = 3;
        if (2.90>xContainer || xContainer>3.10 || 2.90>xPos ||xPos>3.10){      //If not within position
            failTime = millis();
            // Serial.println("//FAILING STEP 4 criteria ");
            Serial.println("Crane is not in position.");
        } else if (millis() > failTime + 1600) {     //This can be changed to something as a function of velocity and position
            step=5;   
            Serial.println("//Step4 passed");
        }
        delay(5000);
    }

    // Move down to ship and turn off electro magnet.
    if (step==5) {
        Serial.println("Step = 5, move downto ship and turn off electro magnet.");  
        Setpoint_y = 1.15;
        if (yPos > 1.15) {
            Serial3.println("M0");  //Drop load
            Setpoint_y = 0.5; // Go back up
            step=7;
            Serial.println("//PATH A TO B DONE!");
        }
        delay(5000);
    }

    return step;
}