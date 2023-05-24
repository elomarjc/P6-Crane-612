// // Include libraries
// #include <Arduino.h>
// #include <PID_v1.h>

// #include "functions.h"
// #include "pinDefinitions.h"

// //// VARIABLES  ////
// unsigned long time;
// unsigned long lastTime;
// double Kp_y = 32.4, Ki_y = 0, Kd_y = 12.96;

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
//   yPID.SetSampleTime(sampletime);
//   yPID.SetOutputLimits(0.1, 0.9);  // Dutcy cycle Range for Motor Drivers
//   // turn the PID on
//   yPID.SetMode(AUTOMATIC);
//   digitalWrite(pin_enable_y, HIGH);

//   // Serial3.println("M0");  // turn off the magnet
//   Serial3.println("M1");  // turn on the magnet
//   Setpoint_y = 0.3;
// }

// void loop() {
//   time = millis();
//   if (time - lastTime >= sampletime) {
//     Input_y = (double)map(analogRead(pin_pos_y), minY, maxY, 0, 133) / 100;
//     double errorY = Setpoint_y - Input_y;

//     if (-0.02 < errorY && errorY < 0.02) {  // turn off motor if within 1 cm of setpoint
//       analogWrite(pin_pwm_y, 0.5 * 255);
//     } else if (errorY < 0) {                // going up, PWM<0.5
//       Kp_y = 32.4, Ki_y = 0, Kd_y = 12.96;  // PID-values for going up
//       yPID.SetTunings(Kp_y, Ki_y, Kd_y, 1);
//       yPID.Compute();
//       // Output_y = (double) map(Input_y*100, -1.33*100, 1.33*100, 0.1*100, 0.9*100) / 100;
//       analogWrite(pin_pwm_y, max(Output_y * 255- abs(minPWMy_up - 0.5) * 255, 0.1 * 255));
//       Serial.print(Output_y + String(";"));
//     } else if (0 < errorY) {         // going down, PWM>0.5
//       Kp_y = 32.4, Ki_y = 0, Kd_y = 12.96;  // PID-values for going down
//       // Kp_y = 1, Ki_y = 0, Kd_y = 0;
//       yPID.SetTunings(Kp_y, Ki_y, Kd_y, 1);
//       yPID.Compute();
//       // Output_y = (double) map(Input_y*100, 0, 1.33*100, minPWMy_down*100, 0.9*100) / 100;
//       Output_y = (double) map(Output_y*100, 0.5*100, 0.9*100, 0.33*100, 0.9*100) / 100;
//       analogWrite(pin_pwm_y, min(Output_y * 255 , 0.9 * 255)); // + abs(minPWMy_down - 0.5) * 255
//       Serial.print(Output_y + String(";"));
//     }
//     // Serial.print(Input_y + String(";"));

//     // Serial.println(String("\t Current position Y: ") + Input_y + String("\t setpoint: ") + Setpoint_y + String("\t PWM Y: ") + Output_y);

//     if (10000 < time) {
//       Setpoint_y = 0.8;
//     }

//     // Serial.println("Input_y: " + String(Input_y) +
//     //                ", Setpoint_y: " + String(Setpoint_y) + ",Output_y: " + String(Output_y) +
//     //                ", Input_y in meters: " + String((double)map(Input_y, minY, maxY, 0, 133) / 100) +
//     //                ", Setpoint_y in meters: " + String((double)map(Setpoint_y, minY, maxY, 0, 133) / 100));
//     lastTime = time;
//   }
// }
