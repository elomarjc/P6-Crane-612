// // Include libraries
// #include <Arduino.h>
// #include <PID_v1.h>

// #include "functions.h"
// #include "pinDefinitions.h"

// //// VARIABLES  ////
// unsigned long time;
// unsigned long sampletimeTHETA = 1;  // [ms]
// unsigned long lastTime;

// // double Kp_x = 2.4, Ki_x = 0, Kd_x = 1.92,
// // double Kp_theta = 22.6, Ki_theta = 0, Kd_theta = 11.3;

// // PID_v1 xPID(&Input_x, &Output_x, &Setpoint_x, Kp_x, Ki_x, Kd_x, DIRECT);
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
//   // newSetpoint_x(2);
//   Setpoint_theta = 0 - angleOffset;

//   // xPID.SetSampleTime(sampletimeXY);
//   thetaPID.SetSampleTime(sampletimeTHETA);

//   // xPID.SetOutputLimits(255 * 0.1, 255 * 0.9);      // Standard PWM Range for Motor Drivers
//   thetaPID.SetOutputLimits(255 * 0.1, 255 * 0.9);  // Standard PWM Range for Motor Drivers

//   // turn the PID on
//   // xPID.SetMode(AUTOMATIC);
//   thetaPID.SetMode(AUTOMATIC);
//   digitalWrite(pin_enable_x, HIGH);

//   // angleCorrection(); // delete after determining angleOffset
//   // Serial.println(String("Angle offset: ") + angleOffset); // delete after determining angleOffset
//   Serial.println (String("Y-position: ") + map(analogRead(pin_pos_y), minY, maxY, 0, 133));
//   Serial3.println("M1");  // turn on the magnet
//   // delay(100000); // delete after determining angleOffset
// }

// void loop() {
//   time = millis();
//   unsigned long timeChange = (time - lastTime);
//   if (timeChange >= min(sampletimeXY,sampletimeTHETA)) {
//     readInput();
//     // xPID.Compute();
//     thetaPID.Compute();
//     analogWrite(pin_pwm_x, Output_theta);

//     // Serial.println(String("X position: ") + (double)map(Input_x, minX, maxX, 0, 400) / 100 + String(", Theta PWM: ") + Output_theta);
//     Serial.print(Input_theta + String(";"));

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