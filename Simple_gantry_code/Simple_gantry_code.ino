#include "C:\Users\andre\OneDrive - Aalborg Universitet\_Universitet\EIT6\_P6\Github\P6-Crane-612\Testing-Phase\pinDefinitions.h"

float velocity = 0.5;
float trolleyPos = analogRead(pin_pos_x);

// Kode herfra er relateret til software endstops (Rettet til at virke med pwm)
int endstop(int pwm, float min, float max, float pos){
    int pwmEndstop = pwm;
    bool dir = 0;

    // Tjek retning (Det her er skrevet for at kranen bevæger sig modsat retning)
    if (pwm < 127) {
        dir = 0;
    }
    else if (pwm > 127) {
        dir = 1;
    }
    
    // Tjek om endstop switch er ramt
    if (pos > max && dir == 1) {
    pwmEndstop = 127;
    }
    if (pos < min && dir == 0) {
    pwmEndstop = 127;
    }

    return pwmEndstop;
}

void setup()
{
  Serial.begin(115200);
  delay(1000); // giving the microcontroller time to fully start
  Serial.println("--- Starting Gantry Crane ---");
  Serial.println(velocity);
  pinMode(pin_enable_y, OUTPUT);
  pinMode(pin_pwm_y, OUTPUT);
}

void loop()
{
  // Serial.println(trolleyPos);
    Serial.println("Starting");
    digitalWrite(pin_enable_y, HIGH);
    velocity = 0.75;
    analogWrite(pin_pwm_y, 255*0.90);
    delay(1000);
    Serial.println("Finished");
    velocity = 0.25;
    analogWrite(pin_pwm_y, 255*0.10);
    delay(1000);
}