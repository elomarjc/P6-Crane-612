// Include libraries
#include <Arduino.h>
#include <PID_v1.h>

#include "functions.h"
#include "pinDefinitions.h"

//// VARIABLES  ////
unsigned long time;
unsigned long lastTimeTHETA;
unsigned long lastTimeXY;

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
  xPID.SetSampleTime(sampletimeXY);
  thetaPID.SetSampleTime(sampletimeTHETA);

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
  Setpoint_theta = 0;
}

void loop() {
  time = millis();

  if (time - lastTimeTHETA >= sampletimeTHETA) {
    Input_theta = getAngleFromHead();
    thetaPID.Compute();
    double errorTHETA = Setpoint_theta - Input_theta;

    if (-1 < errorTHETA && errorTHETA < 1) {
      analogWrite(pin_pwm_x, 0.5 * 255);
    } else if (errorTHETA > 1) {                                                              // going right, PWM<0.5
      analogWrite(pin_pwm_x, min(Output_theta * 255 - abs(0.5 - minPWMx_right), 0.9 * 255));  //(0.6 - 0.5) * 255 + Output_y * 255);
      // Serial.println(min((minPWMx_left - 0.5) * 255 + Output_x * 255, 0.9 * 255));
      // Serial.println((minPWMx_left - 0.5) * 255 + Output_x * 255);
      // Serial.println((minPWMx_left - 0.5) * 255 + String("\t") + Output_x * 255);
      // Serial.println(minPWMx_left - 0.5);
    } else if (errorTHETA < -1) {                                                            // going left, PWM>0.5
      analogWrite(pin_pwm_x, max(Output_theta * 255 + abs(0.5 - minPWMx_left), 0.1 * 255));  //(0.39 - 0.5) * 255 + Output_y * 255);
      // Serial.println(max((minPWMx_right - 0.5) * 255 + Output_x * 255, 0.1 * 255));
      // Serial.println((minPWMx_right - 0.5) * 255 + Output_x * 255);
      // Serial.println((minPWMx_right - 0.5) * 255 + String("\t") + Output_x * 255);
      // Serial.println(minPWMx_right - 0.5);
    }

    // analogWrite(pin_pwm_x, Output_theta * 255);

    // Serial.println("Input_y: " + String(Input_y) +
    //                ", Setpoint_y: " + String(Setpoint_y) + ",Output_y: " + String(Output_y) +
    //                ", Input_y in meters: " + String((double)map(Input_y, minY, maxY, 0, 133) / 100) +
    //                ", Setpoint_y in meters: " + String((double)map(Setpoint_y, minY, maxY, 0, 133) / 100));
    lastTimeTHETA = time;
  }

  // if (time - lastTimeXY >= sampletimeXY) {
  //   Input_x = (double)map(analogRead(pin_pos_x), minX, maxX, 0, 400) / 100;
  //   xPID.Compute();
  //   double errorX = Setpoint_x - Input_x;

  //   // (Output_x-0.5)+(Output_theta-0.5)

  //   if (errorX > 0) {                                                                                                          // going right, PWM<0.5
  //     analogWrite(pin_pwm_x, min((Output_x - 0.5) * 255 + (Output_theta - 0.5) * 255 - abs(0.5 - minPWMx_right), 0.9 * 255));  //(0.6 - 0.5) * 255 + Output_y * 255);
  //     // Serial.println(min((minPWMx_left - 0.5) * 255 + Output_x * 255, 0.9 * 255));
  //     // Serial.println((minPWMx_left - 0.5) * 255 + Output_x * 255);
  //     // Serial.println((minPWMx_left - 0.5) * 255 + String("\t") + Output_x * 255);
  //     // Serial.println(minPWMx_left - 0.5);
  //   } else if (errorX < 0) {                                                                                                  // going left, PWM>0.5
  //     analogWrite(pin_pwm_x, max((Output_x - 0.5) * 255 + (Output_theta - 0.5) * 255 + abs(0.5 - minPWMx_left), 0.1 * 255));  //(0.39 - 0.5) * 255 + Output_y * 255);
  //     // Serial.println(max((minPWMx_right - 0.5) * 255 + Output_x * 255, 0.1 * 255));
  //     // Serial.println((minPWMx_right - 0.5) * 255 + Output_x * 255);
  //     // Serial.println((minPWMx_right - 0.5) * 255 + String("\t") + Output_x * 255);
  //     // Serial.println(minPWMx_right - 0.5);
  //   }

  //   // Serial.println("Input_y: " + String(Input_y) +
  //   //                ", Setpoint_y: " + String(Setpoint_y) + ",Output_y: " + String(Output_y) +
  //   //                ", Input_y in meters: " + String((double)map(Input_y, minY, maxY, 0, 133) / 100) +
  //   //                ", Setpoint_y in meters: " + String((double)map(Setpoint_y, minY, maxY, 0, 133) / 100));
  //   lastTimeXY = time;
  // }

  Serial.println(String("Angle: ") + Input_theta + String("\t PWM angle: ") + Output_theta +
                 String("\t X position: ") + Input_x + String("\t PWM x: ") + Output_x +
                 String("\t PWM sum: ") + ((Output_x - 0.5) + (Output_theta - 0.5)) + String("\t PWM difference: ") + ((Output_x - 0.5) - (Output_theta - 0.5)));

  //   if (20000 < time) {
  //     Setpoint_x = 3;
  //   }
}