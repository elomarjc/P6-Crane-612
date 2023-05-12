// // Include libraries
// #include <Arduino.h>
// #include "pinDefinitions.h"
// #include <PID_v1.h>

// Include libraries
#include <Arduino.h>
#include <functions.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>
#include "sigProc.h"
#include "path.h"
#include "pathVertical.h"
#include "math.h"
#include "pinDefinitions.h"
#include "dataStructures.h"
#include "displayHandler.h"
#include <PID_v1.h>

// Loop sample period
uint32_t Ts = 1e6 / 100; // 100 Hz

#define DYNAMICNOTCHFILTER

low_pass xPosLowpasss = low_pass(0.03); // Lowpass filter tau = 30 ms.
low_pass yPosLowpasss = low_pass(0.03); // Lowpass filter tau = 30 ms.
low_pass angleLowpass = low_pass(0.03); // Lowpass filter tau = 30 ms.
low_pass angleHighpass = low_pass(1);

#ifdef DYNAMICNOTCHFILTER
NotchFilter angleNotchFilter;
#else

/*
H_z =

  0.9697 z^2 - 1.918 z + 0.9697
  -----------------------------
     z^2 - 1.918 z + 0.9394
*/

float b[3] = {0.9412, -1.862, 0.9412};
float a[3] = {1.0000, -1.862, 0.8824};

IIR angleNotchFilter = IIR(a, b);

#endif

// Define Variables we'll be connecting to
double Setpoint_y, Input_y, Output_y,
    Setpoint_x, Input_x, Output_x,
    Setpoint_theta, Input_theta, Output_theta;

bool magnet_sw;

// Specify the links and initial tuning parameters
double Kp_y = 32.4,
       Ki_y = 0,
       Kd_y = 12.96,
       kg_y = 1,

       Kp_x = 2.4,   // 2.4
       Ki_x = 0,
       Kd_x = 1.92,  //1.92
       kg_x = 1,

       Kp_theta = 22.6, //22.6
       Ki_theta = 0,
       Kd_theta = 11.3, //11.3
       kg_theta = 1;

PID_v1 yPID(&Input_y, &Output_y, &Setpoint_y, Kp_y *kg_y, Ki_y *kg_y, Kd_y *kg_y, DIRECT);
PID_v1 xPID(&Input_x, &Output_x, &Setpoint_x, Kp_x *kg_x, Ki_x *kg_x, Kd_x *kg_x, DIRECT);
PID_v1 thetaPID(&Input_theta, &Output_theta, &Setpoint_theta, Kp_theta *kg_theta, Ki_theta *kg_theta, Kd_theta *kg_theta, REVERSE);

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

// Convert wire length to 2nd pendulum frequency
float wirelengthToFrequency(float length, bool withContainer)
{
  if (withContainer)
  {
    return 3.85 - atan(6.5 * length);
  }
  else
  {
    return 2.85 - atan(5 * length) * 0.51;
  }
}

// Function that reads the inputs to the system
void readInput()
{
  Input_x = analogRead(pin_pos_x);
  Input_y = analogRead(pin_pos_y);
  Input_theta = getAngleFromHead();

  // Filter trolley position inputs
  Input_x = xPosLowpasss.update(Input_x);
  Input_y = yPosLowpasss.update(Input_y);

  Input_theta = angleLowpass.update(Input_theta);
  Input_theta = Input_theta - angleHighpass.update(Input_theta);

// Update notch filter parameters
#ifdef DYNAMICNOTCHFILTER
  angleNotchFilter.updateFrequency(wirelengthToFrequency(Input_y, magnet_sw));
#endif

  Input_theta = angleNotchFilter.update(Input_theta);

  // // Sanity check angle data
  // while (Input_theta > 90 || Input_theta < -90)
  // {
  //   digitalWrite(pin_enable_x, LOW); // Stop motors!
  //   digitalWrite(pin_enable_y, LOW);
  //   Serial.println("//Insane angle data");
  //   Input_theta = getAngleFromHead();
  // }
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
  Setpoint_x = 300;
  Setpoint_theta = 0;

  yPID.SetSampleTime(10); // Sample rate expressed in ms, 10ms = 100 Hz
  xPID.SetSampleTime(10);
  thetaPID.SetSampleTime(10);

  xPID.SetOutputLimits(255 * 0.1, 255 * 0.9); // PWM Range for the specific motors
  yPID.SetOutputLimits(255 * 0.1, 255 * 0.9);
  thetaPID.SetOutputLimits(255 * 0.1, 255 * 0.9);

  // turn the PID on
  yPID.SetMode(AUTOMATIC);
  xPID.SetMode(AUTOMATIC);
  thetaPID.SetMode(AUTOMATIC);

  // turn magnet on
  magnet_sw = 1;
  Serial3.println("M1");

#ifdef DYNAMICNOTCHFILTER
  // Initial values
  angleNotchFilter = NotchFilter(2.35, 2, Ts / 1e6);
#endif

  Serial.println("Setup done!");
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

  Serial.println("PID Input_x: " + String(Input_x) + ", PID Setpoint_x: " + String(Setpoint_x) + ", PID Output_x: " + String(Output_x) + ", Angle: " + String(Input_theta) + ", Output_theta " + String(Output_theta) + ", Output_theta+x " + String(Output_x-Output_theta));
}