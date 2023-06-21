// Include libraries
#include <Arduino.h>
#include <PID_v1.h>
#include <functions.h>
#include "sigProc.h"
#include "pinDefinitions.h"

//// VARIABLES  ////
unsigned long time;
unsigned long lastTime;
unsigned long lastTimeTest1;
unsigned long lastTimeTest2;
bool flag = true;
int state = 0;
unsigned long stateTime = 20000;

low_pass xVelLowpass           = low_pass(0.1);           // Lowpass filter tau = 30 ms.
forwardEuler xTrolleyVelCal     = forwardEuler();           // For calculating trolley speed in the y-axis
forwardEuler yTrolleyVelCal     = forwardEuler();           // For calculating trolley speed in the x-axis

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

  // Serial.println(String("Y-position: ") + map(analogRead(pin_pos_y), minY, maxY, 0, 133));
  //   Serial3.println("M0");  // turn off the magnet
  // Serial3.println("M1");  // turn on the magnet
  Setpoint_y = 0.2;
  Setpoint_x = 2;
  Setpoint_theta = 0;
}

void loop()
{
  time = millis();

  if (time - lastTime >= sampletime) {
    unsigned long start = micros();
    //Serial.println(String("Start time: ")+start);

    //readInput();
    Input_x = (double)map(analogRead(pin_pos_x), minX, maxX, 0, 400) / 100;
    Input_theta = getAngleFromHead();
    Input_y = (double)map(analogRead(pin_pos_y), minY, maxY, 0, 133) / 100;

    // Calculate container position
    xContainer = Input_x + (sin(Input_theta)) * Input_y;
    yContainer = (cos((Input_theta * PI) / 180)) * Input_y;

    float trolleyVelocity = xVelLowpass.update(xTrolleyVelCal.update(Input_x));
    float hoistVelocity = yTrolleyVelCal.update(Input_y);

    //pathAtoB(Input_x, Input_y, xContainer, yContainer);
    pathBtoA(Input_x, Input_y, xContainer, yContainer);

    // // Calculate PWM using PID
    if (magnet_sw == 0)
    {
      yPID.SetTunings(6.7, 0, 12.96, 1);
    }
    else
    {
      yPID.SetTunings(6.1, 0, 12.96, 1);   // Kp=2
    }

    xPID.Compute();
    thetaPID.Compute();
    yPID.Compute();

    //// Y-AXIS ////
    double currentY = Output_y;
    if (currentY > 0)
    { // going down, PWM>0.5, Current>0
      // Serial.println(currentY + String(" here1"));

      if (abs(hoistVelocity) < 0.6) {
        currentY = min(currentY + minCurrenty_down, currentLimity_down);
      }
      else {
        currentY = min(currentY, currentLimity_down);
      }

      //currentY = min(currentY + minCurrenty_down, currentLimity_down);

      // Serial.println(currentY + String(" here1"));
      double PWMcurrent = (double)map(currentY * 100, currentLimity_up * 100, currentLimity_down * 100, 0.1 * 100, 0.9 * 100) / 100;
      // Serial.println(currentY + String(" here1 ") + PWMcurrent);
      analogWrite(pin_pwm_y, PWMcurrent * 255);
    }
    else if (currentY < 0)
    { // going up, PWM<0.5, Current<0
      // Serial.println(currentY + String(" here2"));
      
      if (abs(hoistVelocity) < 0.6) {
        currentY = max(currentY + minCurrenty_up, currentLimity_up);
      }
      else {
        currentY = max(currentY, currentLimity_up);
      }
      
      currentY = max(currentY + minCurrenty_up, currentLimity_up);
      // Serial.println(currentY + String(" here2"));
      double PWMcurrent = (double)map(currentY * 100, currentLimity_up * 100, currentLimity_down * 100, 0.1 * 100, 0.9 * 100) / 100;
      // Serial.println(currentY + String(" here2 ") + PWMcurrent);
      analogWrite(pin_pwm_y, PWMcurrent * 255);
    }

    //// X-AXIS ////
    double currentX = Output_x - Output_theta;
    if (currentX > 0)
    { // going left, PWM>0.5
      
      if (abs(trolleyVelocity) < 0.6) {
        currentX = min(currentX + minCurrentx_left, currentLimitx_left);
      }
      else {
        currentX = min(currentX, currentLimitx_left);
      }
    
      //currentX = min(currentX + minCurrentx_left, currentLimitx_left);
      double PWMcurrent = (double)map(currentX * 100, currentLimitx_right * 100, currentLimitx_left * 100, 0.1 * 100, 0.9 * 100) / 100;
      analogWrite(pin_pwm_x, PWMcurrent * 255);
    }
    else if (currentX < 0)
    { // going right, PWM<0.5
      if (abs(trolleyVelocity) < 0.6) {
        currentX = max(currentX + minCurrentx_right, currentLimitx_right);
      }
      else {
        currentX = max(currentX, currentLimitx_right);
      }

      //currentX = max(currentX + minCurrentx_right, currentLimitx_right);
      double PWMcurrent = (double)map(currentX * 100, currentLimitx_right * 100, currentLimitx_left * 100, 0.1 * 100, 0.9 * 100) / 100;
      analogWrite(pin_pwm_x, PWMcurrent * 255);
    }

    lastTime = time;
    // Serial.println(String("total loop time in micros: ") + (micros() - start) );
    // Serial.println(String("angle: ") + Input_theta);

    //Serial.println(String("setpoint X: ") + Setpoint_x + String(", input X: ") + Input_x + String(", x raw: ") + analogRead(pin_pos_x) + String(", setpoint Y: ") + Setpoint_y + String(", input Y: ") + Input_y + String(", angle: ") + Input_theta + String(", y raw: ") + analogRead(pin_pos_y));
    Serial.println(String("Steps: ") + step + String(", Failtime: ") + failTime);
  }
}