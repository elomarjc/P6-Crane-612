#include <Arduino.h>
#line 1 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\Simple_gantry_code\\Simple_gantry_code.ino"
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
int minY = 61;  // ceiling
int maxY = 937; // floor

// Kode herfra er relateret til software endstops (Rettet til at virke med pwm)
#line 19 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\Simple_gantry_code\\Simple_gantry_code.ino"
int endstop(int pwm, float min, float max, float pos);
#line 47 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\Simple_gantry_code\\Simple_gantry_code.ino"
void setVelocityX(float velocity);
#line 52 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\Simple_gantry_code\\Simple_gantry_code.ino"
void setVelocityY(float velocity);
#line 57 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\Simple_gantry_code\\Simple_gantry_code.ino"
void positionalValues();
#line 73 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\Simple_gantry_code\\Simple_gantry_code.ino"
void backAndForth();
#line 84 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\Simple_gantry_code\\Simple_gantry_code.ino"
void verifyPositionX();
#line 106 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\Simple_gantry_code\\Simple_gantry_code.ino"
void verifyCurrentX();
#line 125 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\Simple_gantry_code\\Simple_gantry_code.ino"
void verifyCurrentY();
#line 134 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\Simple_gantry_code\\Simple_gantry_code.ino"
void communicationHead();
#line 144 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\Simple_gantry_code\\Simple_gantry_code.ino"
void angleCorrection();
#line 163 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\Simple_gantry_code\\Simple_gantry_code.ino"
void setup();
#line 182 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\Simple_gantry_code\\Simple_gantry_code.ino"
void loop();
#line 19 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\Simple_gantry_code\\Simple_gantry_code.ino"
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
  analogWrite(pin_pwm_x, 255 * velocity);
}

void setVelocityY(float velocity) // [0.1; 0.9], <0.5 up?, >0.5 down?, =0.5 stand still, speed is determined as difference between 0.5 and given value
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

void verifyPositionX()
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
  communicationHead();
}

void verifyCurrentX()
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

void verifyCurrentY()
{
  digitalWrite(pin_enable_y, HIGH);
  setVelocityY(0.25);

  int currentPos = map(analogRead(pin_pos_y), minY, maxY, 0, 133); // [cm]
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
  Serial.begin(9600);  // communication with microcontroller
  Serial3.begin(9600); // communication with head (the error is ok if the program will compile)
  delay(1000);        // giving the microcontroller time to fully start
  Serial.println("--- Starting Gantry Crane ---");
  pinMode(pin_enable_x, OUTPUT);
  pinMode(pin_pwm_x, OUTPUT);
  pinMode(pin_pos_x, INPUT);
  pinMode(pin_enable_y, OUTPUT);
  pinMode(pin_pwm_y, OUTPUT);
  pinMode(pin_pos_y, INPUT);
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
