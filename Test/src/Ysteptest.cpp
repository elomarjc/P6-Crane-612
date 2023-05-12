// // Include libraries
// #include <Arduino.h>
// #include <PID_v1.h>

// #include "functions.h"
// #include "pinDefinitions.h"

// //// VARIABLES  ////
// unsigned long time;
// unsigned long lastTime;

// PID_v1 yPID(&Input_y, &Output_y, &Setpoint_y, Kp_y, Ki_y, Kd_y, DIRECT);

// void setup() {
//   Serial.begin(115200);
//   Serial3.begin(9600);
//   // Set input pinMode
//   pinMode(pin_pos_y, INPUT);

//   // Set output pinMode
//   pinMode(pin_enable_y, OUTPUT);
//   pinMode(pin_pwm_y, OUTPUT);

//   // define parameters for the PID-controller
//   yPID.SetSampleTime(sampletimeXY);
//   yPID.SetOutputLimits(0.1, 0.9);  // PWM Range for Motor Drivers
//   // turn the PID on
//   yPID.SetMode(AUTOMATIC);
//   digitalWrite(pin_enable_y, HIGH);

//   // Serial3.println("M0");  // turn off the magnet
//   Serial3.println("M1");  // turn on the magnet
//   Setpoint_y = 0.1;
// }

// void loop() {
//   time = millis();
//   if (time - lastTime >= sampletimeXY) {
//     Input_y = (double)map(analogRead(pin_pos_y), minY, maxY, 0, 133) / 100;
//     yPID.Compute();
//     double errorY = Setpoint_y - Input_y;
//     if (errorY < 0) {                                                                        // going up, PWM>0.5
//       analogWrite(pin_pwm_y, max(Output_y * 255, 0.1 * 255 - abs(minPWMy_up - 0.5) * 255));  //(0.6 - 0.5) * 255 + Output_y * 255);
//       // Serial.print((minPWMy_up - 0.5) * 255 + Output_y * 255 + String("; "));
//     } else if (0 < errorY) {                                                                 // going down, PWM<0.5
//       analogWrite(pin_pwm_y, min(Output_y * 255, 0.9 * 255) - abs(minPWMy_up - 0.5) * 255);  //(0.39 - 0.5) * 255 + Output_y * 255);
//       // Serial.print((minPWMy_down - 0.5) * 255 + Output_y * 255 + String("; "));
//     }
//     // Serial.print(Input_y + String(";"));

//     Serial.println(String("\t Current position Y: ") + Input_y + String("\t setpoint: ") + Setpoint_y + String("\t PWM Y: ") + Output_y);

//     // if (20000 < time) {
//     //   Setpoint_y = 1.1;
//     // }

//     // Serial.println("Input_y: " + String(Input_y) +
//     //                ", Setpoint_y: " + String(Setpoint_y) + ",Output_y: " + String(Output_y) +
//     //                ", Input_y in meters: " + String((double)map(Input_y, minY, maxY, 0, 133) / 100) +
//     //                ", Setpoint_y in meters: " + String((double)map(Setpoint_y, minY, maxY, 0, 133) / 100));
//     lastTime = time;
//   }
// }
