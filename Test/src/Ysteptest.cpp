/*
// Include libraries
#include <Arduino.h>
#include <PID_v1.h>

#include "pinDefinitions.h"

//// VARIABLES  ////
int minY = 50;   // ceiling
int maxY = 873;  // floor

// Define Variables we'll be connecting to
double Setpoint_y, Input_y, Output_y;

// Specify the links and initial tuning parameters
double Kp_y = 32.4, Ki_y = 0, Kd_y = 12.96;

PID_v1 yPID(&Input_y, &Output_y, &Setpoint_y, Kp_y, Ki_y, Kd_y, DIRECT);

void newSetpoint_y(double newSetpoint) {
  Setpoint_y = (double)map(newSetpoint * 100, 0, 133, minY, maxY);
}

// Function that reads the inputs to the system
// and makes convertions
void readInput() {
  Input_y = analogRead(pin_pos_y);
}

void setup() {
  Serial.begin(115200);
  // Set input pinMode
  pinMode(pin_pos_y, INPUT);

  // Set output pinMode
  pinMode(pin_enable_y, OUTPUT);
  pinMode(pin_pwm_y, OUTPUT);

  // initialize the variables we're linked to
  // Input_y = analogRead(pin_pos_y);
  newSetpoint_y(0.5);  // [m]
  // Setpoint_y = 0.5;  // [m]

  yPID.SetSampleTime(10);

  yPID.SetOutputLimits(0.1 * 255, 0.9 * 255);  // Standard PWM Range for Motor Drivers

  // turn the PID on
  yPID.SetMode(AUTOMATIC);
  digitalWrite(pin_enable_y, HIGH);
}

void loop() {
  readInput();
  yPID.Compute();

  analogWrite(pin_pwm_y, Output_y);

  Serial.println("Input_y: " + String(Input_y) + ", Setpoint_y: " + String(Setpoint_y) + ", Output_y: " + String(Output_y) + ", PWM: " + String(Output_y) + ", Input_y in meters: " + String((double)map(Input_y, 50, 873, 0, 133) / 100) + ", Setpoint_y in meters: " + String((double)map(Setpoint_y, 50, 873, 0, 133) / 100));
}
*/

/*
// Include libraries
#include <Arduino.h>
#include <PID_v1.h>

#include "pinDefinitions.h"

//// VARIABLES  ////
int minY = 50;   // ceiling
int maxY = 873;  // floor

// Define Variables we'll be connecting to
double Setpoint_y, Input_y, Output_y;

// Specify the links and initial tuning parameters
double Kp_y = 32.4, Ki_y = 0, Kd_y = 12.96;

PID_v1 yPID(&Input_y, &Output_y, &Setpoint_y, Kp_y, Ki_y, Kd_y, DIRECT);

float goodMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// void newSetpoint_y(double newSetpoint) {
//   Setpoint_y = (double)map(newSetpoint * 100, 0, 133, 50, 873);
// }

// Function that reads the inputs to the system
// and makes convertions
void readInput() {
  Input_y = goodMap(analogRead(pin_pos_y), minY, maxY, 0, 1.33);
}

void setup() {
  Serial.begin(115200);
  // Set input pinMode
  pinMode(pin_pos_y, INPUT);

  // Set output pinMode
  pinMode(pin_enable_y, OUTPUT);
  pinMode(pin_pwm_y, OUTPUT);

  // initialize the variables we're linked to
  // Input_y = analogRead(pin_pos_y);
  // newSetpoint_y(0.5);
  Setpoint_y = 0.5;  // [m]

  yPID.SetSampleTime(10);

  yPID.SetOutputLimits(0.1, 0.9);  // Standard PWM Range for Motor Drivers

  // turn the PID on
  yPID.SetMode(AUTOMATIC);
  digitalWrite(pin_enable_y, HIGH);
}

void loop() {
  readInput();
  yPID.Compute();

  analogWrite(pin_pwm_y, 255 * Output_y);

  Serial.println("Input_y: " + String(Input_y) + ", Setpoint_y: " + String(Setpoint_y) + ", Output_y: " + String(Output_y) + ", PWM: " + String(255 * Output_y));
  // + ", Input_y in meters: " + String((double)map(Input_y, 50, 873, 0, 133) / 100) + ", Setpoint_y in meters: " + String((double)map(Setpoint_y, 50, 873, 0, 133) / 100));
}
*/

/*
// Include libraries
#include <Arduino.h>
#include <PID_v1.h>

#include "pinDefinitions.h"

//// VARIABLES  ////
unsigned long time;
unsigned long sampletimeXY = 10;  // [ms]
unsigned long lastTime;
int minY = 50;   // ceiling
int maxY = 873;  // floor

// Define Variables we'll be connecting to
double Setpoint_y, Input_y, Output_y;

// Specify the links and initial tuning parameters
double Kp_y = 32.4, Ki_y = 0, Kd_y = 12.96;

PID_v1 yPID(&Input_y, &Output_y, &Setpoint_y, Kp_y, Ki_y, Kd_y, DIRECT);

void newSetpoint_y(double newSetpoint) {
  Setpoint_y = (double)map(newSetpoint * 100, 0, 133, minY, maxY);
}

// Function that reads the inputs to the system and makes convertions
void readInput() {
  Input_y = analogRead(pin_pos_y);
}

void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);
  // Set input pinMode
  pinMode(pin_pos_y, INPUT);

  // Set output pinMode
  pinMode(pin_enable_y, OUTPUT);
  pinMode(pin_pwm_y, OUTPUT);

  // initialize the variables we're linked to
  newSetpoint_y(0.1);

  yPID.SetSampleTime(sampletimeXY);

  yPID.SetOutputLimits(255 * 0.1, 255 * 0.9);  // Standard PWM Range for Motor Drivers

  // turn the PID on
  yPID.SetMode(AUTOMATIC);
  digitalWrite(pin_enable_y, HIGH);

  Serial3.println("M1"); // turn on the magnet
}

void loop() {
  time = millis();
  unsigned long timeChange = (time - lastTime);
  if (timeChange >= sampletimeXY) {
    readInput();
    yPID.Compute();
    analogWrite(pin_pwm_y, Output_y);

    Serial.print((double)map(Input_y, minY, maxY, 0, 133) / 100 + String(";"));

    if (20000 < time) {
      newSetpoint_y(1.1);
    }

    // Serial.println("Input_y: " + String(Input_y) +
    //                ", Setpoint_y: " + String(Setpoint_y) + ",Output_y: " + String(Output_y) +
    //                ", Input_y in meters: " + String((double)map(Input_y, minY, maxY, 0, 133) / 100) +
    //                ", Setpoint_y in meters: " + String((double)map(Setpoint_y, minY, maxY, 0, 133) / 100));
    lastTime = time;
  }
}
*/
