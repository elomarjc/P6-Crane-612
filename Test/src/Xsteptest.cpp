/*
// Include libraries
#include <Arduino.h>
#include <PID_v1.h>

#include "pinDefinitions.h"
#include "functions.h"

//// VARIABLES  ////
unsigned long time;
unsigned long sampletimeXY = 10;  // [ms]
unsigned long lastTime;

// Specify the links and initial tuning parameters
double Kp_x = 2.4, Ki_x = 0, Kd_x = 1.92;

PID_v1 xPID(&Input_x, &Output_x, &Setpoint_x, Kp_x, Ki_x, Kd_x, DIRECT);

void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);
  // Set input pinMode
  pinMode(pin_pos_x, INPUT);

  // Set output pinMode
  pinMode(pin_enable_x, OUTPUT);
  pinMode(pin_pwm_x, OUTPUT);

  // initialize the variables we're linked to
  newSetpoint_x(2);

  xPID.SetSampleTime(sampletimeXY);

  xPID.SetOutputLimits(255 * 0.1, 255 * 0.9);  // Standard PWM Range for Motor Drivers

  // turn the PID on
  xPID.SetMode(AUTOMATIC);
  digitalWrite(pin_enable_x, HIGH);

  Serial3.println("M1"); // turn on the magnet
}

void loop() {
  time = millis();
  unsigned long timeChange = (time - lastTime);
  if (timeChange >= sampletimeXY) {
    readInput();
    xPID.Compute();
    analogWrite(pin_pwm_x, Output_x);

    Serial.println((double)map(Input_x, minX, maxX, 0, 400) / 100 + String("; "));

    // if (20000 < time) {
    //   newSetpoint_x(3);
    // }

    // Serial.println("Input_x: " + String(Input_x) +
    //                ", Setpoint_x: " + String(Setpoint_x) + ",Output_x: " + String(Output_x) +
    //                ", Input_x in meters: " + String((double)map(Input_x, minX, maxX, 0, 133) / 100) +
    //                ", Setpoint_x in meters: " + String((double)map(Setpoint_x, minX, maxX, 0, 133) / 100));
    lastTime = time;
  }
}
*/