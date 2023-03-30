#include "path.h"
#include "Functions.h"

QauyToShip::QauyToShip(){}

void QauyToShip::update(float xPos, float yPos, float xContainer, float containerSpeed, float *xRefpoint, float *yRefpoint, int electroMagnetLED){
    
    //Before start
    if(step==0) {    
        //If at start position
        if(-0.05<xPos && xPos<0.05 && -0.05<yPos && yPos<0.05){
            step=1;
        }
        else{
            Serial.println("Not in start position");
        }
    }

    //Move to above qauy
    if(step==1){
        Serial.print("Woop...");
        *xRefpoint = 0.5;
        if(0.48>xPos || xPos>0.52){
           failTime = millis();
       } 
       else if(millis() > failTime+500){ //If head has been above container for 0.5s 
           step = 2;
           turnOnElectromagnet(true,electroMagnetLED);
       }
    }

    //Lower head onto container
    if(step==2){
        *yRefpoint = 1.21;
        if(yPos > 1.20)        //Have hit container
            step=3;
    }

    //Move to safety point
    if(step==3){
        *xRefpoint = 2.41;
        *yRefpoint = 0.60;
        if( yPos < 0.67){        //Hopefully crane will not be waiting for this
            step=4;
        }
    }

    //Move above ship
    if(step==4){
        *xRefpoint=3.5;
        if(3.46>xContainer || xContainer>4.54){      //If not within position
            failTime = millis();
        }
        else if(millis() > failTime + 0.9) {     //This can be changed to something as a function of velocity and position
            step=5;   
        }
    }

    //Move right above ship
    if(step==5){
        *yRefpoint = 1.17;  //3 cm above ship

        if(yPos > 1.15 && containerSpeed){
            step=6;
        }

    }

    //Move downto ship and turn off electro magnet.
    if(step==6){
        *yRefpoint = 1.21;
        if(yPos > 1.20){
            turnOnElectromagnet(false,electroMagnetLED);
        }
    }

}

void QauyToShip::reset(){
        step=0;
        failTime =0;
}