// // Include libraries
// #include <Arduino.h>
// // #include <PID_v1.h>

// // #include "functions.h"
// #include "pinDefinitions.h"

// //// VARIABLES ////
// unsigned long time;
// unsigned long lastTime;
// const int sampleTime = 10;  // [ms]
// float angleOffset = 0.5;    // [deg]
// // Define these based on values given in positionalValues() in Simple_gantry_code.ino
// int minX = 855;  // left
// int maxX = 44;   // right

// //// PID-CONTROLLER ////
// double input_x, output_x, setPoint_x;
// double Kp_x = 2.4, Ki_x = 0, Kd_x = 0;
// double input_theta, output_theta, setPoint_theta;
// double Kp_theta = 22.6, Ki_theta = 0, Kd_theta = 0;

// //// FUNCTIONS ////
// void xPID() {
//   double error = setPoint_x - input_x;
//   output_x = 0.5 - error * Kp_x;

//   if (output_x < 0.1) {
//     output_x = 0.1;
//   } else if (output_x > 0.9) {
//     output_x = 0.9;
//   }
//   output_x = output_x;
// }

// void thetaPID() {
//   double error = setPoint_theta - input_theta;
//   output_theta = 0.5 + error * Kp_theta;

//   if (output_theta < 0.1) {
//     output_theta = 0.1;
//   } else if (output_theta > 0.9) {
//     output_theta = 0.9;
//   }
// }

// float getAngleFromHead() {
//   float angle;
//   if (Serial3.available() > 0) {
//     String angleData = Serial3.readStringUntil('\n');
//     angle = angleData.toFloat() - angleOffset;  // [deg]
//     // angle = angle*3.141592/180; // converting angle from [deg] to [rad]
//     // Serial.println(angle);
//   }
//   return angle;
// }

// void setup() {
//   Serial.begin(115200);
//   Serial3.begin(9600);
//   // Define pinmodes
//   pinMode(pin_pos_y, INPUT);
//   pinMode(pin_pos_x, INPUT);
//   pinMode(pin_enable_y, OUTPUT);
//   pinMode(pin_enable_x, OUTPUT);
//   pinMode(pin_pwm_y, OUTPUT);
//   pinMode(pin_pwm_x, OUTPUT);

//   digitalWrite(pin_enable_x, HIGH);
//   digitalWrite(pin_enable_y, HIGH);
//   //   Serial3.println("M0");  // turn off the magnet
//   Serial3.println("M1");  // turn on the magnet

//   setPoint_x = 2;
//   setPoint_theta = 0;
// }

// void loop() {
//   time = millis();
//   if (time - lastTime >= sampleTime) {
//     input_x = (double)map(analogRead(pin_pos_x), minX, maxX, 0, 400);
//     input_theta = getAngleFromHead();
//     xPID();
//     thetaPID();
//     // double direction = (0.5 - output_x) - (0.5 - output_theta);
//     // if (direction > 0) {
//     //   analogWrite(pin_pwm_x, 0.5 * 255 + direction * 255 - (0.5 - 0.39) * 255);
//     // } else if (direction < 0) {
//     //   analogWrite(pin_pwm_x, 0.5 * 255 + direction * 255 + (0.59 - 0.5) * 255);
//     // }
//     // analogWrite(pin_pwm_x, 0.5 * 255 + direction * 255);
//     analogWrite(pin_pwm_x, output_x * 255);
//     // Serial.println(String("Output X: ") + output_x + String("\t Output angle: ") + output_theta + String("\t Direction: ") + direction + String("\t PWM: ") + (pin_pwm_x, 0.5 * 255 + direction * 255));
//     lastTime = time;
//   }
// }