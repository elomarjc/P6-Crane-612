// Include libraries
#include <Arduino.h>
#include <functions.h>
#include <Wire.h>
#include "sigProc.h"
#include "path.h"
#include "pathVertical.h"
#include "math.h"
#include "pinDefinitions.h"
#include "dataStructures.h"
#include <PID_v1.h>

// Loop sample period
uint32_t Ts = 1e6 / 100; // 100 Hz
bool magnet_sw;

#define DYNAMICNOTCHFILTER

low_pass xPosLowpasss = low_pass(0.03); // Lowpass filter tau = 30 ms.
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

void readInput()
{
  Input_x = analogRead(pin_pos_x);
  Input_theta = getAngleFromHead();

  // Filter trolley position inputs
  Input_x = xPosLowpasss.update(Input_x);

  Input_theta = angleLowpass.update(Input_theta);
  Input_theta = Input_theta - angleHighpass.update(Input_theta);

  // Update notch filter parameters
  #ifdef DYNAMICNOTCHFILTER
    angleNotchFilter.updateFrequency(wirelengthToFrequency(Input_y, magnet_sw));
  #endif

  Input_theta = angleNotchFilter.update(Input_theta);
  Input_x = (double)map(Input_x, minX, maxX, 0, 400) / 100;
}

//  Input_x = (double)map(analogRead(pin_pos_x), minX, maxX, 0, 400) / 100;
//  Input_theta = getAngleFromHead();

//// VARIABLES  ////
unsigned long time;
unsigned long lastTime;
unsigned long lastTimeTest1;
unsigned long lastTimeTest2;
bool flag = true;

PID_v1 yPID(&Input_y, &Output_y, &Setpoint_y, Kp_y, Ki_y, Kd_y, DIRECT);
PID_v1 xPID(&Input_x, &Output_x, &Setpoint_x, Kp_x, Ki_x, Kd_x, REVERSE);                              // wire is put on backwards
PID_v1 thetaPID(&Input_theta, &Output_theta, &Setpoint_theta, Kp_theta, Ki_theta, Kd_theta, REVERSE);  // reverse = match xPID

void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);
  // Set input pinMode
  pinMode(pin_pos_y, INPUT);
  pinMode(pin_pos_x, INPUT);

  // Set output pinMode
  pinMode(pin_enable_y, OUTPUT);
  pinMode(pin_enable_x, OUTPUT);
  pinMode(pin_pwm_y, OUTPUT);
  pinMode(pin_pwm_x, OUTPUT);

  // initialize the variables we're linked to
  yPID.SetSampleTime(sampletime);
  xPID.SetSampleTime(sampletime);
  thetaPID.SetSampleTime(sampletime);

  yPID.SetOutputLimits(currentLimity_up, currentLimity_down);         // Current range on motor driver
  xPID.SetOutputLimits(currentLimitx_right, currentLimitx_left);      // Current range on motor driver
  thetaPID.SetOutputLimits(currentLimitx_right, currentLimitx_left);  // Current range on motor driver

  // turn the PID on
  yPID.SetMode(AUTOMATIC);
  xPID.SetMode(AUTOMATIC);
  thetaPID.SetMode(AUTOMATIC);
  digitalWrite(pin_enable_y, HIGH);
  digitalWrite(pin_enable_x, HIGH);

  // angleCorrection();
  // delay(10000);

  Serial.println(String("Y-position: ") + map(analogRead(pin_pos_y), minY, maxY, 0, 133));
  //   Serial3.println("M0");  // turn off the magnet
  Serial3.println("M1");  // turn on the magnet
  Setpoint_y = 0.3;
  Setpoint_x = 0.5;
  Setpoint_theta = 0;

  #ifdef DYNAMICNOTCHFILTER
    // Initial values
    angleNotchFilter = NotchFilter(2.35, 2, Ts / 1e6);
  #endif


}

void loop() {
  time = millis();

  if (time - lastTime >= sampletime) {
    //readInput();
    Input_x = (double)map(analogRead(pin_pos_x), minX, maxX, 0, 400) / 100;
    Input_theta = getAngleFromHead();
    Input_y = (double)map(analogRead(pin_pos_y), minY, maxY, 0, 133) / 100;
    xPID.Compute();
    thetaPID.Compute();
    yPID.Compute();

    //// Y-AXIS ////
    double currentY = Output_y;
    if (currentY > 0) {  // going down, PWM>0.5, Current>0
      // yPID.SetTunings(2, 0, 12.96, 1); // the last 1 is bc of PID library - DON't TOUCH
      // yPID.Compute();
      // Serial.println(currentY + String(" here1"));
      currentY = min(currentY + minCurrenty_down, currentLimity_down);
      // Serial.println(currentY + String(" here1"));
      double PWMcurrent = (double)map(currentY * 100, currentLimity_up * 100, currentLimity_down * 100, 0.1 * 100, 0.9 * 100) / 100;
      // Serial.println(currentY + String(" here1 ") + PWMcurrent);
      analogWrite(pin_pwm_y, PWMcurrent * 255);
    } else if (currentY < 0) {  // going up, PWM<0.5, Current<0
      // yPID.SetTunings(2, 0, 12.96, 1); // the last 1 is bc of PID library - DON't TOUCH
      // yPID.Compute();
      // Serial.println(currentY + String(" here2"));
      currentY = max(currentY + minCurrenty_up, currentLimity_up);
      // Serial.println(currentY + String(" here2"));
      double PWMcurrent = (double)map(currentY * 100, currentLimity_up * 100, currentLimity_down * 100, 0.1 * 100, 0.9 * 100) / 100;
      // Serial.println(currentY + String(" here2 ") + PWMcurrent);
      analogWrite(pin_pwm_y, PWMcurrent * 255);
    }

    // Serial.print(Input_y + String(";"));
    // Serial.println(
    //     String("Y position: ") + Input_y +
    //     String("\t Y current: ") + Output_y +
    //     String("\t PWM: ") + ((double)map(currentY * 100, currentLimity_up * 100, currentLimity_down * 100, 0.1 * 100, 0.9 * 100) / 100));

    //// X-AXIS ////
    double currentX = Output_x - Output_theta;
    if (currentX > 0) {  // going left, PWM>0.5
      currentX = min(currentX + minCurrentx_left, currentLimitx_left);
      double PWMcurrent = (double)map(currentX * 100, currentLimitx_right * 100, currentLimitx_left * 100, 0.1 * 100, 0.9 * 100) / 100;
      analogWrite(pin_pwm_x, PWMcurrent * 255);
    } else if (currentX < 0) {  // going right, PWM<0.5
      currentX = max(currentX + minCurrentx_right, currentLimitx_right);
      double PWMcurrent = (double)map(currentX * 100, currentLimitx_right * 100, currentLimitx_left * 100, 0.1 * 100, 0.9 * 100) / 100;
      analogWrite(pin_pwm_x, PWMcurrent * 255);
    }

    Serial.print(Input_theta + String(";"));
    // Serial.println(
    //     String("Angle: ") + Input_theta +
    //     String("\t angle current: ") + Output_theta +
    //     String("\t X position: ") + Input_x +
    //     String("\t X current: ") + Output_x +
    //     String("\t Total current: ") + (Output_x + Output_theta) +
    //     String("\t PWM: ") + ((double)map(currentX * 100, currentLimitx_right * 100, currentLimitx_left * 100, 0.1 * 100, 0.9 * 100) / 100));

    lastTime = time;
  }

  // if (time - lastTimeTest1 >= 20000) {
  //   if (flag) {
  //     Setpoint_x = 0.5;
  //     // Serial.println("here1");
  //   } else {
  //     Setpoint_x = 3.5;
  //     // Serial.println("here2");
  //   }
  //   flag = !flag;
  //   lastTimeTest1 = time;
  // }

  // if (time - lastTimeTest1 >= 10000) {
  //   if (flag) {
  //     Setpoint_y = 1.1;
  //     // Serial.println("here1");
  //   } else {
  //     Setpoint_y = 0.1;
  //     // Serial.println("here2");
  //   }
  //   flag = !flag;
  //   lastTimeTest1 = time;
  // }
}

// void loop() {
//   time = millis();

//   if (time - lastTime >= sampletime) {
//     //// INNER LOOP ////
//     Input_theta = getAngleFromHead();
//     thetaPID.Compute();
//     double errorTHETA = Setpoint_theta - Input_theta;

//     // if (-1 < errorTHETA && errorTHETA < 1) {
//     //   analogWrite(pin_pwm_x, 0.5 * 255);
//     // } else
//     if (errorTHETA > 0) {                                                                     // going right, PWM<0.5
//       analogWrite(pin_pwm_x, min(Output_theta * 255 - abs(0.5 - minPWMx_right), 0.9 * 255));  //(0.6 - 0.5) * 255 + Output_y * 255);
//       // Serial.println(min((minPWMx_left - 0.5) * 255 + Output_x * 255, 0.9 * 255));
//       // Serial.println((minPWMx_left - 0.5) * 255 + Output_x * 255);
//       // Serial.println((minPWMx_left - 0.5) * 255 + String("\t") + Output_x * 255);
//       // Serial.println(minPWMx_left - 0.5);
//     } else if (errorTHETA < 0) {                                                             // going left, PWM>0.5
//       analogWrite(pin_pwm_x, max(Output_theta * 255 + abs(0.5 - minPWMx_left), 0.1 * 255));  //(0.39 - 0.5) * 255 + Output_y * 255);
//       // Serial.println(max((minPWMx_right - 0.5) * 255 + Output_x * 255, 0.1 * 255));
//       // Serial.println((minPWMx_right - 0.5) * 255 + Output_x * 255);
//       // Serial.println((minPWMx_right - 0.5) * 255 + String("\t") + Output_x * 255);
//       // Serial.println(minPWMx_right - 0.5);
//     }

//     // analogWrite(pin_pwm_x, Output_theta * 255);

//     //// OUTER LOOP ////
//     // Input_x = (double)map(analogRead(pin_pos_x), minX, maxX, 0, 400) / 100;
//     // xPID.Compute();
//     // double errorX = Setpoint_x - Input_x;

//     // // (Output_x-0.5)+(Output_theta-0.5)

//     // if (errorX > 0) { // going right, PWM<0.5
//     // //   analogWrite(pin_pwm_x, min((Output_x - 0.5) * 255 + (Output_theta - 0.5) * 255 + 0.5*255 - abs(0.5 - minPWMx_right), 0.9 * 255));  //(0.6 - 0.5) * 255 + Output_y * 255);
//     //   analogWrite(pin_pwm_x, min((Output_x - 0.5) * 255- abs(0.5 - minPWMx_right), 0.9 * 255));  //(0.6 - 0.5) * 255 + Output_y * 255);
//     //   // Serial.println(min((minPWMx_left - 0.5) * 255 + Output_x * 255, 0.9 * 255));
//     //   // Serial.println((minPWMx_left - 0.5) * 255 + Output_x * 255);
//     //   // Serial.println((minPWMx_left - 0.5) * 255 + String("\t") + Output_x * 255);
//     //   // Serial.println(minPWMx_left - 0.5);
//     // } else if (errorX < 0) { // going left, PWM>0.5
//     // //   analogWrite(pin_pwm_x, max((Output_x - 0.5) * 255 + (Output_theta - 0.5) * 255 + 0.5*255 + abs(0.5 - minPWMx_left), 0.1 * 255));  //(0.39 - 0.5) * 255 + Output_y * 255);
//     //   analogWrite(pin_pwm_x, max((Output_x - 0.5) * 255 + abs(0.5 - minPWMx_left), 0.1 * 255));  //(0.39 - 0.5) * 255 + Output_y * 255);
//     //   // Serial.println(max((minPWMx_right - 0.5) * 255 + Output_x * 255, 0.1 * 255));
//     //   // Serial.println((minPWMx_right - 0.5) * 255 + Output_x * 255);
//     //   // Serial.println((minPWMx_right - 0.5) * 255 + String("\t") + Output_x * 255);
//     //   // Serial.println(minPWMx_right - 0.5);
//     // }

//     // Serial.println("Input_y: " + String(Input_y) +
//     //                ", Setpoint_y: " + String(Setpoint_y) + ",Output_y: " + String(Output_y) +
//     //                ", Input_y in meters: " + String((double)map(Input_y, minY, maxY, 0, 133) / 100) +
//     //                ", Setpoint_y in meters: " + String((double)map(Setpoint_y, minY, maxY, 0, 133) / 100));

//     //   // Serial.println("Input_y: " + String(Input_y) +
//     //   //                ", Setpoint_y: " + String(Setpoint_y) + ",Output_y: " + String(Output_y) +
//     //   //                ", Input_y in meters: " + String((double)map(Input_y, minY, maxY, 0, 133) / 100) +
//     //   //                ", Setpoint_y in meters: " + String((double)map(Setpoint_y, minY, maxY, 0, 133) / 100));
//     lastTime = time;
//   }

//   Serial.println(String("Angle: ") + Input_theta + String("\t PWM angle: ") + Output_theta +
//                  String("\t X position: ") + Input_x + String("\t PWM x: ") + Output_x +
//                  String("\t PWM sum: ") + ((Output_x - 0.5) + (Output_theta - 0.5)) + String("\t PWM difference: ") + ((Output_x - 0.5) - (Output_theta - 0.5)));

//   //   if (20000 < time) {
//   //     Setpoint_x = 3;
//   //   }
// }