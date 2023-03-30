#include <Arduino.h>
#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
boolean init_success = false;

#define PIN_MAGNET 12
#define PIN_MAGLED 4
//#define PRINTING

void setup() {

  Serial.begin(9600);
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(PIN_MAGNET,OUTPUT);

  // Initialize MPU
  #ifdef PRINTING
  Serial.println("Setting up MPU");
  #endif
  Wire.begin();
  delay(20);
  mpu.initialize();

  if (mpu.dmpInitialize() == 0){
    #ifdef PRINTING
    Serial.println("Calibrating MPU");
    #endif
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    
    mpu.setDLPFMode(MPU6050_DLPF_BW_188);
    #ifdef PRINTING
    Serial.println("MPU calibration complete");
    #endif
    digitalWrite(LED_BUILTIN,HIGH);
    init_success = true;
  } 
  else {
    #ifdef PRINTING
    Serial.println("MPU setup FAILED!");
    #endif
    init_success = false;
  }
}

void loop() {

  if (init_success){
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      Quaternion q;           // [w, x, y, z]         quaternion container
      VectorFloat gravity;    // [x, y, z]            gravity vector
      float ypr[3];
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.println(ypr[2]*(180/PI));
    }
    if (Serial.available() > 0) {
      String rcv = Serial.readStringUntil(*"\n");

      // Read for magnet command
      if (rcv.indexOf("M1")>-1){
        digitalWrite(PIN_MAGNET,HIGH);
        //digitalWrite(PIN_MAGLED,HIGH);
      } else if (rcv.indexOf("M0")>-1){
        digitalWrite(PIN_MAGNET,LOW);
        //digitalWrite(PIN_MAGLED,LOW);
      }
    } 
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
  }
}