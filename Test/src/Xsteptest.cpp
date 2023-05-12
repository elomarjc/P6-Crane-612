// // Include libraries
// #include <Arduino.h>
// #include <PID_v1.h>

// #include "functions.h"
// #include "pinDefinitions.h"

// //// VARIABLES  ////
// unsigned long time;
// unsigned long lastTimeXY;

// PID_v1 xPID(&Input_x, &Output_x, &Setpoint_x, Kp_x, Ki_x, Kd_x, REVERSE);  // wire is put on backwards

// void setup() {
//   Serial.begin(115200);
//   Serial3.begin(9600);
//   // Set input pinMode
//   pinMode(pin_pos_x, INPUT);

//   // Set output pinMode
//   pinMode(pin_enable_x, OUTPUT);
//   pinMode(pin_pwm_x, OUTPUT);

//   xPID.SetSampleTime(sampletimeXY);

//   xPID.SetOutputLimits(0.1, 0.9);  // PWM Range for Motor Drivers

//   // turn the PID on
//   xPID.SetMode(AUTOMATIC);
//   digitalWrite(pin_enable_x, HIGH);

//   Serial.println(String("Y-position: ") + map(analogRead(pin_pos_y), minY, maxY, 0, 133));
//   // Serial3.println("M0");  // turn off the magnet
//   Serial3.println("M1");  // turn on the magnet
//   Setpoint_x = 2;
// }

// void loop() {
//   time = millis();
//   if (time - lastTimeXY >= sampletimeXY) {
//     Input_x = (double)map(analogRead(pin_pos_x), minX, maxX, 0, 400) / 100;
//     xPID.Compute();
//     double errorX = Setpoint_x - Input_x;

//     if (errorX > 0) {                                                                     // going right, PWM<0.5
//       analogWrite(pin_pwm_x, min(Output_x * 255 - abs(0.5 - minPWMx_right), 0.9 * 255));  //(0.6 - 0.5) * 255 + Output_y * 255);
//       // Serial.println(min((minPWMx_left - 0.5) * 255 + Output_x * 255, 0.9 * 255));
//       // Serial.println((minPWMx_left - 0.5) * 255 + Output_x * 255);
//       // Serial.println((minPWMx_left - 0.5) * 255 + String("\t") + Output_x * 255);
//       // Serial.println(minPWMx_left - 0.5);
//     } else if (errorX < 0) {                                                             // going left, PWM>0.5
//       analogWrite(pin_pwm_x, max(Output_x * 255 + abs(0.5 - minPWMx_left), 0.1 * 255));  //(0.39 - 0.5) * 255 + Output_y * 255);
//       // Serial.println(max((minPWMx_right - 0.5) * 255 + Output_x * 255, 0.1 * 255));
//       // Serial.println((minPWMx_right - 0.5) * 255 + Output_x * 255);
//       // Serial.println((minPWMx_right - 0.5) * 255 + String("\t") + Output_x * 255);
//       // Serial.println(minPWMx_right - 0.5);
//     }

//     Serial.print(Input_x + String(";"));
//     // Serial.println(String("X position: ") + Input_x + String("\t PWM X: ") + Output_x);

//     if (20000 < time) {
//       Setpoint_x = 3;
//     }

//     // Serial.println("Input_x: " + String(Input_x) +
//     //                ", Setpoint_x: " + String(Setpoint_x) + ",Output_x: " + String(Output_x) +
//     //                ", Input_x in meters: " + String((double)map(Input_x, minX, maxX, 0, 133) / 100) +
//     //                ", Setpoint_x in meters: " + String((double)map(Setpoint_x, minX, maxX, 0, 133) / 100));
//     lastTimeXY = time;
//   }
// }