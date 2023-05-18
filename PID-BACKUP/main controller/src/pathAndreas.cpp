//New global variables
int step = 0;
uint16_t failTime =0;

//Make sure to replace
// xPos = x position of trolley
// yPos = y position of trolley
// angle = angle input

//Calculate container position
float xContainer = xPos+(sin((angle*PI)/180))*yPos;
float yContainer = (cos((angle*PI)/180))*yPos;

// new function
int pathAtoB (){
    if (step==0) {    
        Serial.println("Step = 0");
        //If at start position
        if (-0.1<xPos && xPos<0.450 && -0.05<yPos && yPos<0.052){ //put the right values
            step=1;
            Serial.println("Trolley is at the start position");
        } else {
            Serial.println("Not in start position");
        }
        delay(2000);
    } 

    // Move to above qauy
    if (step==1) {
        Setpoint_x = 0.9;
        if (0.885>xPos || xPos>0.915) {    //If trolley is not above container. pm 2 cm
           failTime = millis();
           Serial.println("Trolley is not above container.");
        } else if (millis() > failTime + 300) { //If head has been above container for 0.5s 
           Serial.println("Trolley is above container.");
           step = 2;
           turnOnElectromagnet(true,LelectroMagnetLED);
           Serial.println("//Step1 passed");
       }
       delay(5000);
    }

    // Lower head onto container
    if (step==2) {
        Serial.println("Step = 2, lower head onto container");
        Setpoint_y = 0.8;
        if (yPos<0.73) {
            failTime = millis();
            Serial.println("//Step2 passed");
        } else if (millis() > failTime + 400) {
            step=4;
            Serial.println("Else if step=4");
        }
        delay(5000);
    }

    // Hoist contrainer
    if (step==3) {
        Serial.println("Step = 3, move to safety point");
        Setpoint_y = 1.20;
        if (yPos < 1.12) {        
            step=4;
            Serial.println("//Step3 passed");
        }
        delay(5000);
    }

    // Move above ship
    if (step==4) {
        Serial.println("Step = 4, move above ship");
        Setpoint_x = 3;
        Setpoint_y = 0.8;
        if (2.90>xContainer || xContainer>3.10 || 2.90>xPos ||xPos>3.10){      //If not within position
            failTime = millis();
            // Serial.println("//FAILING STEP 4 criteria ");
            Serial.println("Crane is not in position.");
        } else if (millis() > failTime + 1600) {     //This can be changed to something as a function of velocity and position
            step=5;   
            Serial.println("//Step4 passed");
        }
        delay(5000);
    }


    // Move down to ship and turn off electro magnet.
    if (step==5) {
        Serial.println("Step = 5, move downto ship and turn off electro magnet.");  
        Setpoint_y = 0.280;
        if (yPos > 0.283) {
            turnOnElectromagnet(false,LelectroMagnetLED);
            Setpoint_y = 1.30;
            step=7;
            Serial.println("//Step5 passed");
        }
        delay(5000);
    }

    return step;
}