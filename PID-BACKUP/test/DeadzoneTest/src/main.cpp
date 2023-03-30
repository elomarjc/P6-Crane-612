#include <Arduino.h>

const unsigned int magnetSw = 2;
const unsigned int autoManualSw = 3;

int enableX = 8;
int enableY = 9;
int pwmX = 10;
int pwmY = 11;

void setup() {

  Serial.begin(9600);
  Serial3.begin(9600);
  for(int i = 8; i < 12; i++){
    pinMode(i,OUTPUT);
  }
  pinMode(magnetSw,INPUT);
  pinMode(autoManualSw,INPUT);
  analogWrite(pwmX,127);
  analogWrite(pwmY,127);
}

void loop() {
  for(int i = 128; i < 255; i++){
    analogWrite(pwmY,i);
    Serial.print("Input for analog write function is ");
    Serial.println(i);
    delay(1000);

    if(digitalRead(autoManualSw) == HIGH){
    digitalWrite(enableX,HIGH);
    digitalWrite(enableY,HIGH);
    Serial.println("Go");
  }
  else{
    digitalWrite(enableX,LOW);
    digitalWrite(enableY,LOW);
    Serial.println("Nogo");
    }

  if(digitalRead(magnetSw) == HIGH){
    Serial3.println("M1");
    Serial.println("Magnet toggled");
  }
  else{
    Serial3.println("M0");
    Serial.println("Magnet not toggled");
    }
  }
}