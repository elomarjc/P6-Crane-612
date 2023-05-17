#include "pathVertical.h"
#include "functions.h"
#include "dataStructures.h"

#include <Arduino.h>

QauyToShipV::QauyToShipV(int electroMagnetLED){
    LelectroMagnetLED = electroMagnetLED;
}

int QauyToShipV::update(float xPos, float yPos, xy_float  *ref , float xContainer, float containerSpeed, bool *pathRunning, bool *innnerLoopOn){
       
    // Before start
    if (step==0) {    
        Serial.println("Step = 0");
        //If at start position
        if (-0.1<xPos && xPos<0.450 && -0.05<yPos && yPos<0.052){
            step=1;
            *pathRunning=true;
            *innnerLoopOn = false;
            Serial.println("Trolley is at the start position");
            Serial.println("//Step0 passed");
        } else {
            Serial.println("Not in start position");
        }
        delay(2000);
    } 
    
    if (step==1) {
        Serial.println("Step = 1, move to above quay");
        ref->y = 1.1;
        *innnerLoopOn = false;
        if (yPos<1.05)
        {
            Serial.println("Trolley is above container.");
            step = 2;
            Serial3.println("M1");
            Serial.println("//Step1 passed");
        }
    }

    // Move to above qauy
    if (step==1) {
        Serial.println("Step = 1, move to above quay");
        ref->x = 0.9;
        *innnerLoopOn = false;
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
        ref->y = 0.8;
        *innnerLoopOn = false;
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
        ref->y = 1.20;
        *innnerLoopOn = true;
        if ( yPos < 1.12) {        
            step=4;
            Serial.println("//Step3 passed");
        }
        delay(5000);
    }

    // Move above ship
    if (step==4) {
        Serial.println("Step = 4, move above ship");
        ref->x=3;
        ref->y=0.8;
        *innnerLoopOn = true;
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
        ref->y = 0.280;
        *innnerLoopOn = false;
        if (yPos > 0.283) {
            turnOnElectromagnet(false,LelectroMagnetLED);
            *innnerLoopOn = false;
            ref->y = 1.30;
            step=7;
            Serial.println("//Step5 passed");
        }
        delay(5000);
    }

    return step;
}

void QauyToShipV::reset(){
        step=0;
        failTime =0;
}


//******************* Ship To Qauy*********************
//*****************************************************



ShipToQauyV::ShipToQauyV(int electroMagnetLED){
    LelectroMagnetLED = electroMagnetLED;
}

void ShipToQauyV::update(float xPos, float yPos, xy_float *ref, float xContainer, float containerSpeed,  bool *pathRunning, bool *innerLoopOn){
    
    //Before start
    if(step==0) {    
        //If at start position
        if(3.90<xPos && xPos<4.05 && -0.05<yPos && yPos<0.05){
            step=1;
            *pathRunning=true;
            *innerLoopOn = false;
        }
        else{
            Serial.println("//Not in start position");
        }
    }

    //Move to above ship
    if(step==1){
        ref->x = 3.5;
        *innerLoopOn = false;
        if(3.48>xPos || xPos>3.52){    //If trolley is not above container. pm 2 cm
           failTime = millis();
       } 
       else if(millis() > failTime+300){ //If head has been above container for 0.3s 
           step = 2;
           turnOnElectromagnet(true,LelectroMagnetLED);
       }
    }

    if(step==2){
        ref->y = 1.23;    //Just below top of container
        *innerLoopOn = false;
        if(yPos < 1.21) {       //Have hit container
            failTime=millis();
            
        }
        else if (millis()>failTime+300)
        {
            step=3;
        }
        
    }

    if(step==3){
        ref->y = 0.74;
        *innerLoopOn = false;
        if(yPos<0.80){
            step=4;
        }
    }

    if(step==4){                    
        ref->x = 0.5;
        *innerLoopOn = true;
        if(xPos<3.50-0.23){ //Safty point, Container can be lowered from now on
            step=5;
        }
    }

    if(step==5){
        //ref->y = 1.15; //3cm above qauy
        *innerLoopOn = true;
        if(xContainer<0.44 || xContainer> 0.54||xPos<0.44 || xPos> 0.54 ){
            failTime=millis();
            Serial.print("//Test 5 failed");
        }

        if(millis() >failTime +900  && containerSpeed < 0.45){   //Container not swinging for 0.9 s, low wire speed and les than 6 cm above ground
            step=6;
        }

    }

    if(step==6){
        ref->y = 1.23;
        *innerLoopOn = true;
        if(yPos > 1.21){
            turnOnElectromagnet(false,LelectroMagnetLED);
            ref->y=0.3;        //Move head away from container
            step=7;
        }

    }
}

void ShipToQauyV::reset(){
        step=0;
        failTime =0;
}


