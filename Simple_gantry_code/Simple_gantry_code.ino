#include "pinDefinitions.h"
#include "functions.h"
// #include <Arduino.h>
// #include <Wire.h>
// #include "math.h"

////// VARIABLES //////
float angleOffset = 0;
int startPos = -1;
int finalPos = 0;
int time = millis();
// define these based on values given in positionalValues()
int minX = 132; // left
int maxX = 951; // right
int minY = 4; // ceiling
int maxY = 873; // floor

void setVelocityX(float velocity) // [0.1; 0.9], <0.5 to the left, >0.5 to the right, =0.5 stand still, speed is determined as difference between 0.5 and given value
{
  analogWrite(pin_pwm_x, 255 * velocity);
}

void setVelocityY(float velocity) // [0.1; 0.9], <0.5 down, >0.5 up, =0.5 stand still, speed is determined as difference between 0.5 and given value
{
  analogWrite(pin_pwm_y, 255 * velocity);
}

void positionalValues()
{
  digitalWrite(pin_enable_x, LOW);
  digitalWrite(pin_enable_y, LOW);
  Serial.println("-------------------");
  Serial.print("X position in analogRead: ");
  Serial.println(analogRead(pin_pos_x));
  Serial.print("Y position in analogRead: ");
  Serial.println(analogRead(pin_pos_y));
  Serial.print("X position in cm: ");
  Serial.println(map(analogRead(pin_pos_x), minX, maxX, 0, 400));
  Serial.print("Y position in cm: ");
  Serial.println(map(analogRead(pin_pos_y), minY, maxY, 0, 133));
  delay(1000);
}

void backAndForth()
{
  // Serial.println("Starting");
  digitalWrite(pin_enable_x, HIGH);
  setVelocityX(0.75);
  delay(1000);
  // Serial.println("Finished");
  setVelocityX(0.25);
  delay(1000);
}

void verifyAngleX()
{
  int currentPos = map(analogRead(pin_pos_x), minX, maxX, 0, 400); // [cm]

  if (startPos == -1 || startPos > currentPos)
  {
    startPos = currentPos;
    Serial.print("Starting position: ");
    Serial.println(startPos);
  }

  if (finalPos < currentPos)
  {
    finalPos = currentPos;
    Serial.print("Final position: ");
    Serial.println(finalPos);
    Serial.print("Distance traveled: ");
    Serial.println(finalPos - startPos);
  }
  getAngleFromHead();
}

void verifyPositionX()
{
  digitalWrite(pin_enable_x, HIGH);
  setVelocityX(0.65);

  int currentPos = map(analogRead(pin_pos_x), minX, maxX, 0, 400); // [cm]
  Serial.println(currentPos);

  if (currentPos > 350)
  {
    Serial.println(millis());
  }

  while (currentPos > 350)
  {
    setVelocityX(0.25);
  }
}

void verifyPositionY()
{
  digitalWrite(pin_enable_y, HIGH);
  setVelocityY(0.10);

  int currentPos = map(analogRead(pin_pos_y), minY, maxY, 0, 133); // [cm]
  Serial.println(currentPos);
}

void getAngleFromHead()
{
  if (Serial3.available() && Serial3.read() == '\n')
  {
    String angleData = Serial3.readStringUntil('\n');
    float angle = angleData.toFloat() - angleOffset;
    Serial.println(angle);
  }
}

void angleCorrection()
{
  Serial.println("--- Correcting angle ---");
  float angleSum = 0;
  int n = 1000;
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

void setup()
{
  Serial.begin(9600);  // communication with microcontroller
  Serial3.begin(9600); // communication with head (the error is ok if the program will compile)
  delay(1000);         // giving the microcontroller time to fully start
  Serial.println("--- Starting Gantry Crane ---");
  pinMode(pin_enable_x, OUTPUT);
  pinMode(pin_pwm_x, OUTPUT);
  pinMode(pin_pos_x, INPUT);
  pinMode(pin_enable_y, OUTPUT);
  pinMode(pin_pwm_y, OUTPUT);
  pinMode(pin_pos_y, INPUT);
  setVelocityX(0.5);
  setVelocityY(0.5);
  // angleCorrection();
  time = millis();
  Serial.println(time);
}

void loop()
{
  positionalValues();
  // verifyPositionY();
  // getAngleFromHead();
  // Serial.println(map(analogRead(pin_pos_y), minY, maxY, 0, 133));
  // delay(1000);
  // while (time + 10000 < millis())
  // {
  // }
}