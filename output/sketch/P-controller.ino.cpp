#include <Arduino.h>
#line 1 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\P-controller\\P-controller.ino"
#include "functions.h"
#include "pinDefinitions.h"

////// VARIABLES //////
float K_p = 15;
// float K_p = 15;
// float K_d = 5;
float endPoint;
int time = millis();

#line 11 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\P-controller\\P-controller.ino"
void setup();
#line 46 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\P-controller\\P-controller.ino"
void loop();
#line 11 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\P-controller\\P-controller.ino"
void setup() {
  Serial.begin(9600);   // communication with microcontroller //// MAY NEED TO BE 9600 ////
  Serial3.begin(9600);  // communication with head (the error is ok if the program will compile)
  while (!Serial) {     // empty while loop while waiting for Serial-port to open
  }
  while (!Serial3) {  // empty while loop while waiting for Serial3-port to open
  }
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
  digitalWrite(pin_enable_x, HIGH);
  digitalWrite(pin_enable_y, HIGH);
  endPoint = 0.5;
  // Serial.println("now");
  // delay(10000);
  


  //// KODE TIL TEST ////
  // sæt kranhoved i 0
  // endpoint er 1 meter
  delay(10000);
  Serial.println("Starting test");
  time = millis();
  Serial.println(time);
  // husk at Serial.println() position
}

void loop() {
  // setVelocityY(0.5 + errorY(endPoint) * K_p);

  //// P-CONTROLLER ////
  //   if (0.1 < 0.50 + errorY(endPoint) * K_p && 0.50 + errorY(endPoint) * K_p < 0.9) {
  //     setVelocityY(0.50 + errorY(endPoint) * K_p);
  //   } else if (0.50 + errorY(endPoint) * K_p < 0.1) {
  //     setVelocityY(0.1);
  //   } else if (0.9 < 0.50 + errorY(endPoint) * K_p) {
  //     setVelocityY(0.9);
  //   }

  // if (errorY(endPoint) * K_p)

  //// NEW AND IMPROVED P-CONTROLLER ////
  if (0.1 < goodMap(errorY(endPoint) * K_p, -1.33, 1.33, 0.1, 0.9) && goodMap(errorY(endPoint) * K_p, -1.33, 1.33, 0.1, 0.9) < 0.9) {
    // setVelocityY(0.50 + errorY(endPoint) * K_p);
    setVelocityY(goodMap(errorY(endPoint) * K_p, -1.33, 1.33, 0.1, 0.9));
  } else if (goodMap(errorY(endPoint) * K_p, -1.33, 1.33, 0.1, 0.9) < 0.1) {
    setVelocityY(0.1);
  } else if (0.9 < goodMap(errorY(endPoint) * K_p, -1.33, 1.33, 0.1, 0.9)) {
    setVelocityY(0.9);
  }

  //   float error = (endPoint - analogRead(pin_pos_y)) * K_p;
  //   goodMap(error, 1.33, -1.33, 0.1, 0.9);

  //// PD-CONTROLLER ////
  // float error1; float error2;
  // error2 = error1;
  // error1 = errorY(endPoint);
  // if (0.1 < 0.50 + errorY(endPoint) * K_p + (error2 - error1) * K_d && 0.50 + errorY(endPoint) * K_p + (error2 - error1) * K_d < 0.9)
  // {
  //     setVelocityY(0.50 + errorY(endPoint) * K_p + (error2 - error1) * K_d);
  // }
  // else if (0.50 + errorY(endPoint) * K_p + (error2 - error1) * K_d < 0.1)
  // {
  //     setVelocityY(0.1);
  // }
  // else if (0.9 < 0.50 + errorY(endPoint) * K_p + (error2 - error1) * K_d)
  // {
  //     setVelocityY(0.9);
  // }

  //// PI-CONTROLLER ////
  // if (0.1 < 0.50 + errorY(endPoint) * K_p && 0.50 + errorY(endPoint) * K_p < 0.9)
  // {
  //     setVelocityY(0.50 + errorY(endPoint) * K_p);
  // }
  // else if (0.50 + errorY(endPoint) * K_p < 0.1)
  // {
  //     setVelocityY(0.1);
  // }
  // else if (0.9 < 0.50 + errorY(endPoint) * K_p)
  // {
  //     setVelocityY(0.9);
  // }

  // Serial.println(0.50 + errorY(0.50) * K_p);
  // Serial.println(errorY(0.50) * K_p);
  // Serial.println(millis());
  Serial.print("Current position: ");
  Serial.println(currentPosY());
  Serial.print("Error: ");
  Serial.println(errorY(endPoint));
  // while (time + 10000 < millis()) {
  // }

  if (time + 10000 < millis() && millis() < time + 20000) {
    endPoint = 0.20;
  } else if (time + 20000 < millis() && millis() < time + 30000) {
    endPoint = 1.2;
  }
}
