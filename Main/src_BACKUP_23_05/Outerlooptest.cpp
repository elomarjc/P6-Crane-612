// // Include libraries
// #include <Arduino.h>
// #include <PID_v1.h>

// #include "functions.h"
// #include "pinDefinitions.h"

// //// VARIABLES  ////
// unsigned long time;
// unsigned long lastTime;

// // double Kp_x = 2.4, Ki_x = 0, Kd_x = 1.92,
// // double Kp_theta = 22.6, Ki_theta = 0, Kd_theta = 11.3;

// PID_v1 xPID(&Input_x, &Output_x, &Setpoint_x, Kp_x, Ki_x, Kd_x, DIRECT);
// PID_v1 thetaPID(&Input_theta, &Output_theta, &Setpoint_theta, Kp_theta, Ki_theta, Kd_theta, REVERSE);

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
//   newSetpoint_x(0.5);
//   Setpoint_theta = 0 - angleOffset;

//   xPID.SetSampleTime(sampletimeXY);
//   thetaPID.SetSampleTime(sampletimeTHETA);

//   xPID.SetOutputLimits(255 * 0.1, 255 * 0.9);      // Standard PWM Range for Motor Drivers
//   thetaPID.SetOutputLimits(255 * 0.1, 255 * 0.9);  // Standard PWM Range for Motor Drivers

//   // turn the PID on
//   xPID.SetMode(AUTOMATIC);
//   thetaPID.SetMode(AUTOMATIC);
//   digitalWrite(pin_enable_x, HIGH);
//   digitalWrite(pin_enable_y, HIGH);

//   // angleCorrection();
//   // delay(10000);

//   Serial.println(String("Y-position: ") + map(analogRead(pin_pos_y), minY, maxY, 0, 133));
//   Serial3.println("M1");  // turn on the magnet
// }

// void loop() {
//   time = millis();
//   unsigned long timeChange = (time - lastTime);
//   if (timeChange >= min(sampletimeXY, sampletimeTHETA)) { //// SPLIT UP INTO ONE FOR X/Y AND ONE FOR THETA ////
//     // readInput();
//     // xPID.Compute();
//     // thetaPID.Compute();
//     //
//     //
//     //
//     int range = map(25, 0, 400, minX, maxX);  // range around Setpoint_x that angle is taken into account (range is 1st argument in map()) [cm]

//     readInput();
//     if (Setpoint_x - range < Input_x && Input_x < Setpoint_x + range) {
//       xPID.Compute();
//       thetaPID.Compute();
//       if (0.1 * 255 < Output_x + Output_theta && Output_x + Output_theta < 0.9 * 255) {
//         analogWrite(pin_pwm_x, Output_x + Output_theta);
//         // Serial.println("1");
//       } else if (Output_x + Output_theta < 0.1 * 255) {
//         analogWrite(pin_pwm_x, 255 * 0.1);
//         // Serial.println("2");
//       } else if (0.9 * 255 < Output_x + Output_theta) {
//         analogWrite(pin_pwm_x, 255 * 0.9);
//         // Serial.println("3");
//       }
//     } else {
//       xPID.Compute();
//       analogWrite(pin_pwm_x, Output_x);
//     }

//     // analogWrite(pin_pwm_x, Output_theta);

//     Serial.println(String("X position: ") + (double)map(Input_x, minX, maxX, 0, 400) / 100 + String(", Total PWM: ") + Output_x + Output_theta);

//     if (20000 < time) {
//       newSetpoint_x(3.5);
//     }

//     // Serial.println("Input_y: " + String(Input_y) +
//     //                ", Setpoint_y: " + String(Setpoint_y) + ",Output_y: " + String(Output_y) +
//     //                ", Input_y in meters: " + String((double)map(Input_y, minY, maxY, 0, 133) / 100) +
//     //                ", Setpoint_y in meters: " + String((double)map(Setpoint_y, minY, maxY, 0, 133) / 100));
//     lastTime = time;
//   }
// }