// Include libraries
#include <Arduino.h>
#include <functions.h>
#include "pinDefinitions.h"
#include <PID_v1.h>

//// VARIABLES  ////
unsigned long time;
unsigned long lastTime;
unsigned long lastTimeTest1;
unsigned long lastTimeTest2;
bool flag = true;
int state = 0;
unsigned long stateTime = 20000;

PID_v1 yPID(&Input_y, &Output_y, &Setpoint_y, Kp_y, Ki_y, Kd_y, DIRECT);
PID_v1 xPID(&Input_x, &Output_x, &Setpoint_x, Kp_x, Ki_x, Kd_x, REVERSE);                             // wire is put on backwards
PID_v1 thetaPID(&Input_theta, &Output_theta, &Setpoint_theta, Kp_theta, Ki_theta, Kd_theta, REVERSE); // reverse = match xPID

//// FOR Y-AXIS ////
void newSetpoint_y(double newSetpoint)
{
  Setpoint_y = newSetpoint;
}

//// FOR X-AXIS ////
void newSetpoint_x(double newSetpoint)
{
  Setpoint_y = 0.3;
  if (Input_y < 0.5)
  {
    Setpoint_x = newSetpoint;
  }
}

void setup()
{
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
  yPID.SetSampleTime(sampletime);
  xPID.SetSampleTime(sampletime);
  thetaPID.SetSampleTime(sampletime);

  yPID.SetOutputLimits(currentLimity_up, currentLimity_down);        // Current range on motor driver
  xPID.SetOutputLimits(currentLimitx_right, currentLimitx_left);     // Current range on motor driver
  thetaPID.SetOutputLimits(currentLimitx_right, currentLimitx_left); // Current range on motor driver

  // turn the PID on
  yPID.SetMode(AUTOMATIC);
  xPID.SetMode(AUTOMATIC);
  thetaPID.SetMode(AUTOMATIC);
  digitalWrite(pin_enable_y, HIGH);
  digitalWrite(pin_enable_x, HIGH);

  Serial3.println("M0");  // turn off the magnet
  magnet_sw = 0;
  //angleCorrection();
  //delay(5000);

  Serial.println(String("Y-position: ") + map(analogRead(pin_pos_y), minY, maxY, 0, 133));
  // Serial3.println("M1");  // turn on the magnet
  Setpoint_y = 0.5;
  Setpoint_x = 0.3;
  Setpoint_theta = 0;
}

void loop()
{
  time = millis();

  if (time - lastTime >= sampletime)
  {
    // readInput();

    Input_x = (double)map(analogRead(pin_pos_x), minX, maxX, 0, 400) / 100;
    Input_theta = getAngleFromHead();
    Input_y = (double)map(analogRead(pin_pos_y), minY, maxY, 0, 133) / 100;

    // Calculate container position
    xContainer = Input_x + (sin(Input_theta)) * Input_y;
    yContainer = (cos((Input_theta * PI) / 180)) * Input_y;

    pathAtoB(Input_x, Input_y, xContainer, yContainer);
    //pathBtoA(Input_x, Input_y, xContainer, yContainer);

    // Calculate PWM using PID
    if (magnet_sw == 0)
    {
      yPID.SetTunings(6.1, 0, 12.96, 1);
      //xPID.SetTunings(1.59, 0, 1.15, 1);
      //thetaPID.SetTunings(2, 0, 0.45, 1);
    }
    else
    {
      yPID.SetTunings(3, 0, 12.96, 1);   // Kp=2
      //xPID.SetTunings(1.59, 0, 1.15, 1); // Kp=1.59
    }

    xPID.Compute();
    thetaPID.Compute();
    yPID.Compute();

    //// Y-AXIS ////
    double currentY = Output_y;
    if (currentY > 0)
    { // going down, PWM>0.5, Current>0
      // Serial.println(currentY + String(" here1"));
      currentY = min(currentY + minCurrenty_down, currentLimity_down);
      // Serial.println(currentY + String(" here1"));
      double PWMcurrent = (double)map(currentY * 100, currentLimity_up * 100, currentLimity_down * 100, 0.1 * 100, 0.9 * 100) / 100;
      // Serial.println(currentY + String(" here1 ") + PWMcurrent);
      analogWrite(pin_pwm_y, PWMcurrent * 255);
    }
    else if (currentY < 0)
    { // going up, PWM<0.5, Current<0
      // Serial.println(currentY + String(" here2"));
      currentY = max(currentY + minCurrenty_up, currentLimity_up);
      // Serial.println(currentY + String(" here2"));
      double PWMcurrent = (double)map(currentY * 100, currentLimity_up * 100, currentLimity_down * 100, 0.1 * 100, 0.9 * 100) / 100;
      // Serial.println(currentY + String(" here2 ") + PWMcurrent);
      analogWrite(pin_pwm_y, PWMcurrent * 255);
    }

    // Serial.print(Input_y + String(";"));
    // Serial.println(
    //     String("Y position: ") + Input_y +
    //     String("\t Y current: ") + Output_y +
    //     String("\t PWM: ") + ((double)map(currentY * 100, currentLimity_up * 100, currentLimity_down * 100, 0.1 * 100, 0.9 * 100) / 100));

    //// X-AXIS ////
    double currentX = Output_x - Output_theta;
    if (currentX > 0)
    { // going left, PWM>0.5
      currentX = min(currentX + minCurrentx_left, currentLimitx_left);
      double PWMcurrent = (double)map(currentX * 100, currentLimitx_right * 100, currentLimitx_left * 100, 0.1 * 100, 0.9 * 100) / 100;
      analogWrite(pin_pwm_x, PWMcurrent * 255);
    }
    else if (currentX < 0)
    { // going right, PWM<0.5
      currentX = max(currentX + minCurrentx_right, currentLimitx_right);
      double PWMcurrent = (double)map(currentX * 100, currentLimitx_right * 100, currentLimitx_left * 100, 0.1 * 100, 0.9 * 100) / 100;
      analogWrite(pin_pwm_x, PWMcurrent * 255);
    }

    Serial.println(String("x pos: ") + String(Input_x) + String(", ") + String("x container: ") + String(xContainer) + String(", ") + String("y pos: ") + String(Input_y) + String(", ") + String("y set: ") + String(Setpoint_y) + String(", ") + String("x set: ") + String(Setpoint_x)+ String(", ")+String("Angle: ") + String((Input_theta*180)/PI));
    
    
    // Serial.println(
    //     String("Angle: ") + Input_theta +
    //     String("\t angle current: ") + Output_theta +
    //     String("\t X position: ") + Input_x +
    //     String("\t X current: ") + Output_x +
    //     String("\t Total current: ") + (Output_x + Output_theta) +
    //     String("\t PWM: ") + ((double)map(currentX * 100, currentLimitx_right * 100, currentLimitx_left * 100, 0.1 * 100, 0.9 * 100) / 100));

    lastTime = time;
  }

  // if (time - lastTimeTest1 >= 20000) {
  //   if (flag) {
  //     Setpoint_x = 0.5;
  //     // Serial.println("here1");
  //   } else {
  //     Setpoint_x = 3.5;
  //     // Serial.println("here2");
  //   }
  //   flag = !flag;
  //   lastTimeTest1 = time;
  // }

  // if (time - lastTimeTest1 >= 10000) {
  //   if (flag) {
  //     Setpoint_y = 1.1;
  //     // Serial.println("here1");
  //   } else {
  //     Setpoint_y = 0.1;
  //     // Serial.println("here2");
  //   }
  //   flag = !flag;
  //   lastTimeTest1 = time;
  // }

  // switch (state) {
  //   case 0:
  //     // collectload();
  //     Setpoint_y = 0.2;
  //     Setpoint_x = 1;
  //     Serial3.println("M1");  // turn on the magnet
  //     if (millis() > (state + 1) * stateTime) {
  //       state++;
  //     }
  //     break;
  //   case 1:
  //     newSetpoint_x(2);
  //     if (millis() > (state + 1) * stateTime) {
  //       state++;
  //     }
  //     break;
  //   case 2:
  //     Setpoint_y = 1.2;
  //     if (millis() > (state + 1) * stateTime) {
  //       dropload();
  //       Setpoint_y = 0.2;
  //       Setpoint_x = 1;
  //       state = 0;
  //     }
  //     break;
  // }
}