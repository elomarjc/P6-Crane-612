// // Include libraries
// #include <Arduino.h>
// #include <PID_v1.h>

// #include "functions.h"
// #include "pinDefinitions.h"

// //// VARIABLES  ////
// unsigned long time;
// unsigned long lastTime;

// PID_v1 thetaPID(&Input_theta, &Output_theta, &Setpoint_theta, Kp_theta, Ki_theta, Kd_theta, DIRECT);  // wire is put on backwards

// void setup() {
//   Serial.begin(115200);
//   Serial3.begin(9600);
//   // Set input pinMode
//   pinMode(pin_pos_y, INPUT);
//   pinMode(pin_pos_x, INPUT);

//   // Set output pinMode
//   pinMode(pin_enable_y, OUTPUT);
//   pinMode(pin_enable_x, OUTPUT);
//   pinMode(pin_pwm_y, OUTPUT);
//   pinMode(pin_pwm_x, OUTPUT);

//   // initialize the variables we're linked to
//   //   angleCorrection();
//   //   delay(10000);
//   Setpoint_theta = 0;

//   thetaPID.SetSampleTime(sampletime);

//   thetaPID.SetOutputLimits(0.1, 0.9);  // Standard PWM Range for Motor Drivers

//   // turn the PID on
//   // xPID.SetMode(AUTOMATIC);
//   thetaPID.SetMode(AUTOMATIC);
//   digitalWrite(pin_enable_x, HIGH);

//   Serial.println(String("Y-position: ") + map(analogRead(pin_pos_y), minY, maxY, 0, 133));
//   Serial3.println("M1");  // turn on the magnet
// }

// void loop() {
//   time = millis();
//   if (time - lastTime >= sampletime) {
//     Input_theta = getAngleFromHead()/100;
//     thetaPID.Compute();
//     double errorTHETA = Setpoint_theta - Input_theta;
//     if (-2 < errorTHETA && errorTHETA < 2) {
//       analogWrite(pin_pwm_x, 0.5 * 255);
//     } else if (errorTHETA > 0) {  // going right, PWM<0.5
//       analogWrite(pin_pwm_x, min(Output_theta * 255 + abs(0.5 - minPWMx_right) * 255, 0.9 * 255));
//       //   analogWrite(pin_pwm_x, max(Output_theta * 255 + abs(0.5 - minPWMx_right) * 255, 0.1 * 255));
//       Serial.println(String("Right ") + min(Output_theta * 255 + abs(0.5 - minPWMx_right) * 255, 0.9 * 255));
//     } else if (errorTHETA < 0) {  // going left, PWM>0.5
//       analogWrite(pin_pwm_x, max(Output_theta * 255 - abs(0.5 - minPWMx_left) * 255, 0.1 * 255));
//       //   analogWrite(pin_pwm_x, min(Output_theta * 255 - abs(0.5 - minPWMx_left) * 255, 0.9 * 255));
//       Serial.println(String("Left") + max(Output_theta * 255 - abs(0.5 - minPWMx_left) * 255, 0.1 * 255));
//     }
//     // else {
//     // analogWrite(pin_pwm_x, Output_theta * 255);
//     // }

//     // Serial.println(String("X position: ") + (double)map(Input_x, minX, maxX, 0, 400) / 100 + String("\t Angle ") + Input_theta + String("\t Theta PWM: ") + Output_theta);
//     // Serial.print(Input_theta + String(";"));

//     // if (20000 < time) {
//     //   newSetpoint_y(1.1);
//     // }

//     // Serial.println("Input_y: " + String(Input_y) +
//     //                ", Setpoint_y: " + String(Setpoint_y) + ",Output_y: " + String(Output_y) +
//     //                ", Input_y in meters: " + String((double)map(Input_y, minY, maxY, 0, 133) / 100) +
//     //                ", Setpoint_y in meters: " + String((double)map(Setpoint_y, minY, maxY, 0, 133) / 100));
//     lastTime = time;
//   }
// }