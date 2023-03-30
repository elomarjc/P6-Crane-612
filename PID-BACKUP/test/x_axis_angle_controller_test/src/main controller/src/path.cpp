#include "path.h"
#include "functions.h"
#include "dataStructures.h"

#include <Arduino.h>

QauyToShip::QauyToShip(int electroMagnetLED){
    LelectroMagnetLED = electroMagnetLED;
}

void QauyToShip::update(float xPos, float yPos, xy_float  *ref , float xContainer, float containerSpeed, bool *pathRunning){
    
    //Before start
    if(step==0) {    
        //If at start position
        if(-0.05<xPos && xPos<0.15 && -0.05<yPos && yPos<0.05){
            step=1;
            *pathRunning=true;
        }
        else{
            Serial.println("Not in start position");
        }
    }

    //Move to above qauy
    if(step==1){
        ref->x = 0.5;
        ref->y = 0.00;
        if(0.48>xPos || xPos>0.52){    //If trolley is not above container. pm 2 cm
           failTime = millis();
       } 
       else if(millis() > failTime+300){ //If head has been above container for 0.5s 
           step = 2;
           turnOnElectromagnet(true,LelectroMagnetLED);
       }
    }

    //Lower head onto container
    if(step==2){
        ref->y = 1.22;
        if(yPos<1.20){
            failTime = millis();
        }

        if(millis() > failTime+200)
            step=3;
    }

    //Move to safety point
    if(step==3){
        ref->x = 2.41;
        ref->y = 0.74;
        if( yPos < 0.80){        //Hopefully crane will not be waiting for this
            step=4;
        }
    }

    //Move above ship
    if(step==4){
        ref->x=3.5;
        if(3.46>xContainer || xContainer>3.54){      //If not within position
            failTime = millis();
        }
        else if(millis() > failTime + 2.0) {     //This can be changed to something as a function of velocity and position
            step=6;   
        }
    }

    //Move right above ship
    if(step==5){
        ref->y = 1.21;  //3 cm above ship

        if(yPos > 1.14 && containerSpeed<0.45){     // Less than 6 cm above graound and speed low
            step=6;
        }

    }

    //Move downto ship and turn off electro magnet.
    if(step==6){
        ref->y = 1.23;
        if(yPos > 1.21){
            turnOnElectromagnet(false,LelectroMagnetLED);
            ref->y = 0.3;
            step=7;
        }
    }
}

void QauyToShip::reset(){
        step=0;
        failTime =0;
}


//******************* Ship To Qauy*********************
//*****************************************************



ShipToQauy::ShipToQauy(int electroMagnetLED){
    LelectroMagnetLED = electroMagnetLED;
}

void ShipToQauy::update(float xPos, float yPos, xy_float *ref, float xContainer, float containerSpeed){
    
    //Before start
    if(step==0) {    
        //If at start position
        if(3.95<xPos && xPos<4.05 && -0.05<yPos && yPos<0.05){
            step=1;
        }
        else{
            Serial.println("Not in start position");
        }
    }

    //Move to above ship
    if(step==1){
        ref->x = 0.5;
        if(3.48>xPos || xPos>3.52){    //If trolley is not above container. pm 2 cm
           failTime = millis();
       } 
       else if(millis() > failTime+300){ //If head has been above container for 0.3s 
           step = 2;
           turnOnElectromagnet(true,LelectroMagnetLED);
       }
    }

    if(step==2){
        ref->y = 1.21;    //Just below top of container
        if(yPos > 1.20) {       //Have hit container
            step=3;
        }
    }

    if(step==3){
        ref->y = 0.74;
        if(yPos<0.80){
            step=4;
        }
    }

    if(step==4){                    
        ref->x = 0.5;
        if(xPos<3.50-0.23){ //Safty point, Container can be lowered from now on
            step=5;
        }
    }

    if(step==5){
        ref->y = 1.15; //3cm above qauy
        if(xContainer<0.44 || xContainer> 0.54){
            failTime=millis();
        }

        if(millis() >failTime + 0.9 && containerSpeed < 0.45){   //Container not swinging for 0.9 s, low wire speed and les than 6 cm above ground
            step=6;
        }

    }

    if(step==6){
        ref->y = 1.21;
        if(yPos > 1.20){
            turnOnElectromagnet(false,LelectroMagnetLED);
            ref->y=0.3;        //Move head away from container
            step=7;
        }

    }
}


