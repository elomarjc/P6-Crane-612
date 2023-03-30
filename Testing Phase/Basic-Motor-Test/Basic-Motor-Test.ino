// Include libraries
#include <Arduino.h>
#include "pinDefinitions.h"

void setup()
{
  // Initialize serial
  // Serial3.begin(9600);  // Communication with head
  Serial.begin(115200); // Communication with PC

  Serial.println("--- Starting Gantry Crane ---");

  // // Set input pinMode
  // pinMode(pin_pos_x, INPUT);
  // pinMode(pin_pos_y, INPUT);

  // Set output pinMode
  pinMode(pin_enable_x, OUTPUT);
  pinMode(pin_enable_y, OUTPUT);
  pinMode(pin_pwm_x, OUTPUT);
  pinMode(pin_pwm_y, OUTPUT);

  // digitalWrite(pin_enable_x, HIGH);
  // digitalWrite(pin_enable_y, HIGH);
}

// Place trolley in center
void loop()
{
  //// X-axis motor
  Serial.println("Starting X-AXIS TRIAL!");
  digitalWrite(pin_enable_x, HIGH);
  if (digitalRead(pin_enable_x) == HIGH)
  { // Is motor on?
    for (int i = 1; i <= 3; i++)
    {                                      // Loop 3 times
      analogWrite(pin_pwm_x, (0.1 * 255)); // Move left, 10% duty cycle
      delay(1000);
      analogWrite(pin_pwm_x, (0.9 * 255)); // Move right, 90% duty cycle
      delay(1050);
      Serial.print("Loop: ");
      Serial.println(i);
    }
  }
  digitalWrite(pin_enable_x, LOW); // Turn motor off
  Serial.println("DONE WITH X-AXIS TRIAL!");
  delay(5000);

  //// Y-axis motor
  Serial.println("Starting Y-AXIS TRIAL!");
  digitalWrite(pin_enable_y, HIGH);
  if (digitalRead(pin_enable_y) == HIGH)
  { // Is motor on?
    for (int i = 1; i <= 3; i++)
    {                                      // Loop 3 times
      analogWrite(pin_pwm_y, (0.9 * 255)); // Move DOWN, 90% duty cycle
      delay(1000);
      analogWrite(pin_pwm_y, (0.1 * 255)); // Move UP, 10% duty cycle
      delay(1400);
      Serial.print("Loop: ");
      Serial.println(i);
    }
  }
  digitalWrite(pin_enable_y, LOW); // Turn motor off
  Serial.println("DONE WITH Y-AXIS TRIAL!");
  delay(4000);
}