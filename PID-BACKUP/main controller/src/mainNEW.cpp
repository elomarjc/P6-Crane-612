// Include libraries
#include <Arduino.h>
#include "pinDefinitions.h"
#include <PID_v1.h>

// Define Variables we'll be connecting to
double Setpoint_y, Input_y, Output_y,
    Setpoint_x, Input_x, Output_x,
    Setpoint_theta, Input_theta, Output_theta;

// Specify the links and initial tuning parameters
double Kp_y = 32.4,
       Ki_y = 0,
       Kd_y = 12.96,
       kg_y = 1,

       Kp_x = 9.82, //2.4
       Ki_x = 0,
       Kd_x = 1.92, // 1.92
       kg_x = 1,

       Kp_theta = 22.6,
       Ki_theta = 0,
       Kd_theta = 11.3,
       kg_theta = 1;

PID_v1 yPID(&Input_y, &Output_y, &Setpoint_y, Kp_y *kg_y, Ki_y *kg_y, Kd_y *kg_y, DIRECT);
PID_v1 xPID(&Input_x, &Output_x, &Setpoint_x, Kp_x *kg_x, Ki_x *kg_x, Kd_x *kg_x, DIRECT);
PID_v1 thetaPID(&Input_theta, &Output_theta, &Setpoint_theta, Kp_theta *kg_theta, Ki_theta *kg_theta, Kd_theta *kg_theta, DIRECT);

float getAngleFromHead()
{
  float angle;
  if (Serial3.available() > 0)
  {
    String angleData = Serial3.readStringUntil('\n');
    angle = angleData.toFloat();
    // Serial.println(angle);
  }
  return angle;
}

// Function that reads the inputs to the system
void readInput()
{
  Input_y = analogRead(pin_pos_y);
  Input_x = analogRead(pin_pos_x);

  Input_theta = getAngleFromHead();

  // Sanity check angle data
  while (Input_theta > 90 || Input_theta < -90)
  {
    digitalWrite(pin_enable_x, LOW); // Stop motors!
    digitalWrite(pin_enable_y, LOW);
    Serial.println("//Insane angle data");
    Input_theta = getAngleFromHead();
  }
}

void setup()
{
  Serial.begin(115200);
  Serial3.begin(9600);

  ; // Set input pinMode
  pinMode(pin_pos_x, INPUT);
  pinMode(pin_pos_y, INPUT);

  // Set output pinMode
  pinMode(pin_enable_x, OUTPUT);
  pinMode(pin_enable_y, OUTPUT);
  pinMode(pin_pwm_x, OUTPUT);
  pinMode(pin_pwm_y, OUTPUT);

  // Initialize the variables we're linked to
  Input_y = analogRead(pin_pos_y);
  Input_x = analogRead(pin_pos_x);

  Setpoint_y = 400;
  Setpoint_x = 600;
  Setpoint_theta = 0;

  yPID.SetSampleTime(10); // Sample rate expressed in ms, 10ms = 100 Hz
  xPID.SetSampleTime(10);
  thetaPID.SetSampleTime(1); // Angle sampling rate is 10x times faster than xPID

  xPID.SetOutputLimits(255 * 0.1, 255 * 0.9); // PWM Range for the specific motors
  yPID.SetOutputLimits(255 * 0.1, 255 * 0.9);
  thetaPID.SetOutputLimits(255 * 0.1, 255 * 0.9);

  // turn the PID on
  yPID.SetMode(AUTOMATIC);
  xPID.SetMode(AUTOMATIC);
  thetaPID.SetMode(AUTOMATIC);

  Serial.println("Setup done!");
  Serial3.println("M1");
}

void loop()
{
  readInput();
  yPID.Compute();
  xPID.Compute();
  thetaPID.Compute();

  digitalWrite(pin_enable_y, HIGH); // Turns on motor
  digitalWrite(pin_enable_x, HIGH);
  analogWrite(pin_pwm_y, Output_y);
  analogWrite(pin_pwm_x, Output_x-Output_theta);

  // int range = 25;
  // if (Setpoint_x - range <= Input_x && Input_x <= Setpoint_x + range)
  // {
  //   analogWrite(pin_pwm_x, Output_x - Output_theta); 
  //   Serial.println("WHY ARE YOU THERE?");
  // }
  // else
  // {
  //   analogWrite(pin_pwm_x, Output_x);
  // }

  Serial.println("PID Input_x: " + String(Input_x) + ", PID Setpoint_x: " + String(Setpoint_x) + ", PID Output_x: " + String(Output_x) + ", Angle: " + String(Input_theta) + ", Kp_y: " + String(Kp_y));
}
