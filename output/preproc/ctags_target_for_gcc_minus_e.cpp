# 1 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\Simple_gantry_code\\Simple_gantry_code.ino"
# 2 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\Simple_gantry_code\\Simple_gantry_code.ino" 2
# 3 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\Simple_gantry_code\\Simple_gantry_code.ino" 2
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
int minY = 61; // ceiling
int maxY = 937; // floor

// Kode herfra er relateret til software endstops (Rettet til at virke med pwm)
int endstop(int pwm, float min, float max, float pos)
{
  int pwmEndstop = pwm;
  bool dir = 0;

  // Tjek retning (Det her er skrevet for at kranen bevæger sig modsat retning)
  if (pwm < 127)
  {
    dir = 0;
  }
  else if (pwm > 127)
  {
    dir = 1;
  }

  // Tjek om endstop switch er ramt
  if (pos > max && dir == 1)
  {
    pwmEndstop = 127;
  }
  if (pos < min && dir == 0)
  {
    pwmEndstop = 127;
  }

  return pwmEndstop;
}

void setVelocityX(float velocity) // [0.1; 0.9], <0.5 to the left, >0.5 to the right, =0.5 stand still, speed is determined as difference between 0.5 and given value
{
  analogWrite(10 /* PWM pin driver x*/, 255 * velocity);
}

void setVelocityY(float velocity) // [0.1; 0.9], <0.5 up?, >0.5 down?, =0.5 stand still, speed is determined as difference between 0.5 and given value
{
  analogWrite(11 /* PWM pin driver y*/, 255 * velocity);
}

void positionalValues()
{
  digitalWrite(8 /* Enable driver x*/, 0x0);
  digitalWrite(9 /* Enable driver y*/, 0x0);
  Serial.println("-------------------");
  Serial.print("X position in analogRead: ");
  Serial.println(analogRead(A0 /* Input from x-axis potentiometer*/));
  Serial.print("Y position in analogRead: ");
  Serial.println(analogRead(A1 /* Input from y-axis potentiometer*/));
  Serial.print("X position in cm: ");
  Serial.println(map(analogRead(A0 /* Input from x-axis potentiometer*/), minX, maxX, 0, 400));
  Serial.print("Y position in cm: ");
  Serial.println(map(analogRead(A1 /* Input from y-axis potentiometer*/), minY, maxY, 0, 133));
  delay(1000);
}

void backAndForth()
{
  // Serial.println("Starting");
  digitalWrite(8 /* Enable driver x*/, 0x1);
  setVelocityX(0.75);
  delay(1000);
  // Serial.println("Finished");
  setVelocityX(0.25);
  delay(1000);
}

void verifyPositionX()
{
  int currentPos = map(analogRead(A0 /* Input from x-axis potentiometer*/), minX, maxX, 0, 400); // [cm]

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
  communicationHead();
}

void verifyCurrentX()
{
  digitalWrite(8 /* Enable driver x*/, 0x1);
  setVelocityX(0.65);

  int currentPos = map(analogRead(A0 /* Input from x-axis potentiometer*/), minX, maxX, 0, 400); // [cm]
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

void verifyCurrentY()
{
  digitalWrite(9 /* Enable driver y*/, 0x1);
  setVelocityY(0.25);

  int currentPos = map(analogRead(A1 /* Input from y-axis potentiometer*/), minY, maxY, 0, 133); // [cm]
  Serial.println(currentPos);
}

void communicationHead()
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
  int i = 0;
  while (i < 100)
  {
    if (Serial3.available() && Serial3.read() == '\n')
    {
      String angleData = Serial3.readStringUntil('\n');
      angleSum += angleData.toFloat();
      i++;
    }
  }
  angleOffset = angleSum / 100;
  Serial.print("Angle offset: ");
  Serial.println(angleOffset);
}

void setup()
{
  Serial.begin(9600); // communication with microcontroller
  Serial3.begin(9600); // communication with head (the error is ok if the program will compile)
  delay(1000); // giving the microcontroller time to fully start
  Serial.println("--- Starting Gantry Crane ---");
  pinMode(8 /* Enable driver x*/, 0x1);
  pinMode(10 /* PWM pin driver x*/, 0x1);
  pinMode(A0 /* Input from x-axis potentiometer*/, 0x0);
  pinMode(9 /* Enable driver y*/, 0x1);
  pinMode(11 /* PWM pin driver y*/, 0x1);
  pinMode(A1 /* Input from y-axis potentiometer*/, 0x0);
  setVelocityX(0.5);
  setVelocityY(0.5);
  angleCorrection();
  time = millis();
  Serial.println(time);
}

void loop()
{
  // verifyPositionX();
  positionalValues();
  // Serial.println(map(analogRead(pin_pos_y), minY, maxY, 0, 133));
  // delay(1000);
  // while (time + 3000 < millis())
  // {
  //   digitalWrite(pin_enable_x, LOW);
  // }
}
