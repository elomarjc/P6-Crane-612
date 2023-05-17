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

PID_v1 xPID(&Input_x, &Output_x, &Setpoint_x, Kp_x, Ki_x, Kd_x, REVERSE);  // wire is put on backwards
PID_v1 thetaPID(&Input_theta, &Output_theta, &Setpoint_theta, Kp_theta, Ki_theta, Kd_theta, DIRECT);

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
  xPID.SetSampleTime(sampletime);
  thetaPID.SetSampleTime(sampletime);

  xPID.SetOutputLimits(0.1, 0.9);      // PWM Range for Motor Drivers
  thetaPID.SetOutputLimits(0.1, 0.9);  // PWM Range for Motor Drivers

  // turn the PID on
  xPID.SetMode(AUTOMATIC);
  thetaPID.SetMode(AUTOMATIC);
  digitalWrite(pin_enable_x, HIGH);
  digitalWrite(pin_enable_y, HIGH);

  // angleCorrection();
  // delay(10000);

  Serial.println(String("Y-position: ") + map(analogRead(pin_pos_y), minY, maxY, 0, 133));
  //   Serial3.println("M0");  // turn off the magnet
  Serial3.println("M1");  // turn on the magnet
  Setpoint_x = 2;
  Setpoint_theta = 0; //-angleOffset

  #ifdef DYNAMICNOTCHFILTER
    // Initial values
    angleNotchFilter = NotchFilter(2.35, 2, Ts / 1e6);
  #endif


}

void loop() {
  time = millis();
  double errorX = Setpoint_x - Input_x;

  if (time - lastTime >= sampletime) {
    readInput();
    xPID.Compute();
    thetaPID.Compute();
    double errorX = Setpoint_x - Input_x;

    //// OUTER LOOP ////
    double direction = (Output_x - 0.5) * 255 + (Output_theta - 0.5) * 255;
    if (direction > 0) {  // going left, PWM>0.5
      double dir = min(0.5*255 + direction + abs(0.5 - minPWMx_right) * 255, 0.9 * 255);
      analogWrite(pin_pwm_x, dir);
      Serial.println(String("left ") + dir);
    } else if (direction < 0) {  // going right, PWM<0.5
      double dir = max(0.5*255 +  direction - abs(0.5 - minPWMx_left) * 255, 0.1 * 255);
      analogWrite(pin_pwm_x, dir);
      Serial.println(String("right ") + dir);
    }

    // if (errorX < 0) {  // going left, PWM>0.5
    //       double dir = min( - abs(0.5 - minPWMx_right) * 255, 0.9 * 255);
    //       analogWrite(pin_pwm_x, dir);
    //       Serial.println(String("left ") + dir);
    //     } else if (errorX > 0) {  // going right, PWM<0.5
    //       double dir = max((Output_x - 0.5) * 255 + (Output_theta - 0.5) * 255 + abs(0.5 - minPWMx_left) * 255, 0.1 * 255);
    //       analogWrite(pin_pwm_x, dir);
    //       Serial.println(String("right ") + dir);
    //     }
    // Serial.println("Input_y: " + String(Input_y) +
    //                ", Setpoint_y: " + String(Setpoint_y) + ",Output_y: " + String(Output_y) +
    //                ", Input_y in meters: " + String((double)map(Input_y, minY, maxY, 0, 133) / 100) +
    //                ", Setpoint_y in meters: " + String((double)map(Setpoint_y, minY, maxY, 0, 133) / 100));

    //// INNER LOOP ////
    // Input_theta = getAngleFromHead();
    // thetaPID.Compute();
    // double errorTHETA = Setpoint_theta - Input_theta;

    // if (-1 < errorTHETA && errorTHETA < 1) {
    //   analogWrite(pin_pwm_x, 0.5 * 255);
    // } else
    // if (errorTHETA > 0) {                                                                     // going right, PWM<0.5
    //   analogWrite(pin_pwm_x, min(Output_theta * 255 - abs(0.5 - minPWMx_right), 0.9 * 255));  //(0.6 - 0.5) * 255 + Output_y * 255);
    // Serial.println(min((minPWMx_left - 0.5) * 255 + Output_x * 255, 0.9 * 255));
    // Serial.println((minPWMx_left - 0.5) * 255 + Output_x * 255);
    // Serial.println((minPWMx_left - 0.5) * 255 + String("\t") + Output_x * 255);
    // Serial.println(minPWMx_left - 0.5);
    // } else if (errorTHETA < 0) {                                                             // going left, PWM>0.5
    //   analogWrite(pin_pwm_x, max(Output_theta * 255 + abs(0.5 - minPWMx_left), 0.1 * 255));  //(0.39 - 0.5) * 255 + Output_y * 255);
    // Serial.println(max((minPWMx_right - 0.5) * 255 + Output_x * 255, 0.1 * 255));
    // Serial.println((minPWMx_right - 0.5) * 255 + Output_x * 255);
    // Serial.println((minPWMx_right - 0.5) * 255 + String("\t") + Output_x * 255);
    // Serial.println(minPWMx_right - 0.5);
    // }

    // analogWrite(pin_pwm_x, Output_theta * 255);

    // Input_x = (double)map(analogRead(pin_pos_x), minX, maxX, 0, 400) / 100;
    // xPID.Compute();
    // double errorX = Setpoint_x - Input_x;

    // // (Output_x-0.5)+(Output_theta-0.5)
    lastTime = time;
  }

  // Serial.println(String("Angle: ") + Input_theta + String("\t PWM angle: ") + Output_theta +
  //                String("\t X position: ") + Input_x + String("\t PWM x: ") + Output_x +
  //                String("\t PWM sum: ") + ((Output_x - 0.5) + (Output_theta - 0.5)) + String("\t PWM difference: ") + ((Output_x - 0.5) - (Output_theta - 0.5)));

  //   if (20000 < time) {
  //     Setpoint_x = 3;
  //   }
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