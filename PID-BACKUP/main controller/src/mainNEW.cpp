// Include libraries
#include <Arduino.h>
#include "pinDefinitions.h"
#include <PID_v1.h>
#include <functions.h>
#include <dataStructures.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>
#include "sigProc.h"
#include "path.h"
#include "pathVertical.h"
#include "math.h"
#include "pinDefinitions.h"
#include "dataStructures.h"
#include "displayHandler.h"
#include <PID_v1.h>


// Define Variables we'll be connecting to
double Setpoint_y, Input_y, Output_y, 
       Setpoint_x, Input_x, Output_x,
       Setpoint_theta, Input_theta, Output_theta;

// Specify the links and initial tuning parameters
double Kp_y = 9.68, Ki_y = 0, Kd_y = 0, 
       Kp_x = 9.68, Ki_x = 0, Kd_x = 0,
       Kp_theta = 1, Ki_theta = 0, Kd_theta = 0;

PID_v1 yPID(&Input_y, &Output_y, &Setpoint_y, Kp_y, Ki_y, Kd_y, DIRECT);
PID_v1 xPID(&Input_x, &Output_x, &Setpoint_x, Kp_x, Ki_x, Kd_x, DIRECT);
PID_v1 thetaPID(&Input_theta, &Output_theta, &Setpoint_theta, Kp_theta, Ki_theta, Kd_theta, DIRECT);

float angleOffset = 0;
int step = 0;

float getAngleFromHead()
{
  float angle;
  if (Serial3.available() && Serial3.read() == '\n')
  {
    String angleData = Serial3.readStringUntil('\n');
    angle = angleData.toFloat() - angleOffset;
    //Serial.println(angle);
  }
  return angle;
}

void angleCorrection()
{
  Serial.println("--- Correcting angle ---");
  float angleSum = 0;
  int n = 100;
  int i = 0;
  while (i < n)
  {
    if (Serial3.available() && Serial3.read() == '\n')
    {
      String angleData = Serial3.readStringUntil('\n');
      angleSum += angleData.toFloat();
      i++;
    }
  }
  angleOffset = angleSum / n;
  Serial.print("Angle offset: ");
  Serial.println(angleOffset);
}


// Function that reads the inputs to the system and makes convertions
void readInput() {
    Input_y = analogRead(pin_pos_y); // 1.3-(0.0015*analogRead(pin_pos_y)-0.17)
    Input_x = analogRead(pin_pos_x);
    
    Input_theta = getAngleFromHead();
    
    // if(Input_theta>90 || Input_theta<-90){         //Sanity check angle data
    //    while(true){
    //        digitalWrite(pin_enable_x, LOW);        //Stop motors!
    //        digitalWrite(pin_enable_y,LOW);
    //        Serial.println("//Insane angle data");
    //    }
    // }

}

void setup()
{
    Serial.begin(115200);
    Serial3.begin(9600);
;    // Set input pinMode
    pinMode(pin_pos_x, INPUT);
    pinMode(pin_pos_y, INPUT);

    // Set output pinMode
    pinMode(pin_enable_x, OUTPUT);
    pinMode(pin_enable_y, OUTPUT);
    pinMode(pin_pwm_x, OUTPUT);
    pinMode(pin_pwm_y, OUTPUT);

    // initialize the variables we're linked to
    Input_y = analogRead(pin_pos_y);
    Input_x = analogRead(pin_pos_x);

    Setpoint_y = 100;
    Setpoint_x = 400;
    Setpoint_theta = 100;
    
    yPID.SetSampleTime(10);
    xPID.SetSampleTime(10);
    thetaPID.SetSampleTime(10);
    
    xPID.SetOutputLimits(255*0.1, 255*0.9); // Standard PWM Range
    yPID.SetOutputLimits(255*0.1, 255*0.9); // Standard PWM Range
    thetaPID.SetOutputLimits(255*0.1, 255*0.9); // Standard PWM Range

    // turn the PID on
    yPID.SetMode(AUTOMATIC);
    xPID.SetMode(AUTOMATIC);
    thetaPID.SetMode(AUTOMATIC);
    angleCorrection();
}

void loop()
{
    readInput();
    yPID.Compute();
    xPID.Compute();
    thetaPID.Compute();
    
    // if (Setpoint_x == 400) {
    //     step = 1;
    // }

    // if (step == 1){
    //     delay(1000);
    //     Serial.println("step 1 PASSED");
    //     Setpoint_x = 650;
    // }

    // if (Input_x == 650) {
    //     step = 0;
    // }


    digitalWrite(pin_enable_y, HIGH);
    digitalWrite(pin_enable_x, HIGH);
    analogWrite(pin_pwm_y, Output_y);
    analogWrite(pin_pwm_x, Output_x);
    //analogWrite(pin_pwm_x, Output_theta);    // the angle is broken



    // // Limit the PWM value to the valid range
    // if (pwm_x > 255*0.9) {
    //     pwm_x = 255*0.9;
    // } else if (pwm_x < 255*0.1) {
    //     pwm_x = 255*0.1;
    // }

    //Serial.println("PID Input_y: " + String(Input_y) + " PID Set point: " + String(Setpoint_y) + " PID Output_y: " + String(Output_y));
    Serial.println("PID Input_x: " + String(Input_x) + " PID Setpoint_x: " + String(Setpoint_x) + " PID Output_x: " + String(Output_x) + " Angle: " + String(Input_theta));
    // Serial.println(Output_y);
}

