// // Include libraries  
// #include <Arduino.h>
// #include <functions.h>
// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
// #include <Adafruit_I2CDevice.h>
// #include "sigProc.h"
// #include "path.h"
// #include "pathVertical.h"
// #include "math.h"
// #include "pinDefinitions.h"
// #include "dataStructures.h"
// #include "displayHandler.h"
// #include <PID_v1.h>

// #define mainprogram
// #ifdef mainprogram

// // Configuration
// #define SAMPLEHZ 100      // Control loop sample frequency
// #define USEPATHALGO      // Uncomment if path algorithm is to be used
// #define DYNAMICNOTCHFILTER

// // x-controller variables - 

// // Outer loop is responsible for moving the trolley to the correct location
// #define xOuterP 1.4    // 1.4 bigger number = pushes harder                          - present
// #define xOuterI 0.75   // 0.75 smaller number = quicker reaction to load changes   - past
// #define xOuterD 1.4    // 1.4 bigger number = dampens oscillation more             - future
// #define xOuterGain 1 // 5.5

// // Inner loop is responsible for keeping the crane head from oscillating.
// #define xInnerP 1.5  // 1.5 bigger number = pushes harder                        - present
// #define xInnerI 0.0  // 0.0 smaller number = quicker reaction to load changes     - past
// #define xInnerD 1    // 1 bigger number = dampens oscillation vmore             - future
// #define xInnerGain 1 // 8

// // y-controller variables
// #define yP 0   // 150   For load on the crane.
// #define yI 0.0     // 0
// #define yD 0    // 75.00
// #define yLP 0.0   // 0.05

// #define noWireP 100 // 100     No-wire for absence of a load on the crane.
// #define noWireI 0  // 0
// #define noWireD 25  // 25

// // Controllers on the x-axis
// // lead_lag xOuterController = lead_lag(xOuterZ ,xOuterP,xOuterG);
// PID xOuterController = PID(xOuterP,xOuterI,xOuterD,0.02);   
// PID xInnerController = PID(xInnerP, xInnerI, xInnerD, 0.05, false);
// PID noWireController = PID(noWireP,noWireI,noWireD,0.04, false);

// // PID controller for y-axis
// PID yController = PID(yP,yI,yD,yLP);

// fastReader reader = fastReader(&Serial3);

// // Loop sample period
// uint32_t Ts = 1e6/SAMPLEHZ;

// // Data struct definitions
// xy_float  ref;
// xy_pwm    pwm;
// inputs    in;

// // Time variables
// uint32_t sampleTimer  = 0;
// uint32_t screenTimer  = 0;
// uint32_t loopTime     = 0;

// #ifdef USEPATHALGO
// //QauyToShip testQuayToShip = QauyToShip(pin_magnet_led);
// QauyToShipV testQuayToShip = QauyToShipV(pin_magnet_led);
// //ShipToQauy testShipToQuay = ShipToQauy(pin_magnet_led);
// #endif

// // Instantiate filters and forward Eulers
// low_pass oledLowpass            = low_pass(0.2);            // Lowpass filter tau = 200 ms.
// low_pass xPosLowpasss           = low_pass(0.03);           // Lowpass filter tau = 30 ms.
// low_pass yPosLowpasss           = low_pass(0.03);           // Lowpass filter tau = 30 ms.
// low_pass angleLowpass           = low_pass(0.03);           // Lowpass filter tau = 30 ms.
// low_pass angleHighpass          = low_pass(1);
// low_pass xVelLowpass           = low_pass(0.1);           // Lowpass filter tau = 30 ms.
// forwardEuler xTrolleyVelCal     = forwardEuler();           // For calculating trolley speed in the y-axis
// forwardEuler yTrolleyVelCal     = forwardEuler();           // For calculating trolley speed in the x-axis
// forwardEuler xContainerVelCal   = forwardEuler();           // For calculating container speed in the x-axis
// forwardEuler yContainerVelCal   = forwardEuler();           // For calculating container speed in the y-axis

// #ifdef DYNAMICNOTCHFILTER
// NotchFilter angleNotchFilter; 
// #else

// /*
// H_z =
 
//   0.9697 z^2 - 1.918 z + 0.9697
//   -----------------------------
//      z^2 - 1.918 z + 0.9394
// */


// float b[3] = {0.9412, -1.862, 0.9412};
// float a[3] = {1.0000, -1.862, 0.8824};

// IIR angleNotchFilter = IIR(a, b);

// #endif

// bool pathRunning = false;
// bool noWireControllerOff = true;

// int state;
// int flag=0;        //makes sure that the serial only prints once the state
// int stateStop=0;

// // Run on startup
// void setup() {

//     // Set safe default reference values
//     #ifndef USEPATHALGO
//     ref.x = .5;  ref.y = .75;
//     #endif

//     // Set input pinMode
//     // pinMode(pin_joystick_x,   INPUT);
//     // pinMode(pin_joystick_y,   INPUT);
//     // pinMode(pin_joystick_sw,  INPUT);
//     // pinMode(pin_magnet_sw,    INPUT);
//     // pinMode(pin_ctrlmode_sw,  INPUT);
//     pinMode(pin_pos_x,        INPUT);
//     pinMode(pin_pos_y,        INPUT);

//     // Set output pinMode
//     pinMode(pin_enable_x,     OUTPUT);
//     pinMode(pin_enable_y,     OUTPUT);
//     pinMode(pin_pwm_x,        OUTPUT);
//     pinMode(pin_pwm_y,        OUTPUT);
//     pinMode(pin_ctrlmode_led, OUTPUT);
//     pinMode(pin_magnet_led,   OUTPUT);

//     in.ctrlmodeSw = 1;

//     // Initialize serial
//     Serial3.begin(9600);  // Communication with head
//     Serial.begin(115200); // Communication with PC
    
//     while(!Serial); 
//     Serial.println("Serial is ready to accept input");
    
//     #ifdef DYNAMICNOTCHFILTER
//     // Initial values
//     angleNotchFilter = NotchFilter(2.35,2,Ts/1e6);
//     #endif

// }

// // Convert wire length to 2nd pendulum frequency
// float wirelengthToFrequency(float length,bool withContainer){
//     if (withContainer){
//         return 3.85-atan(6.5*length);
//     } else {
//         return 2.85-atan(5*length)*0.51;
//     }
// }

// // Convert wire length to 2nd pendulum frequency
// float wirelengthToFrequency2(float length){
//     return 0.1702*length + 1.9084;
// }

// float tempAngle;
// float keyboard_xPos = 0;
// float keyboard_yPos = 0;
// bool keyboard_magnet_sw = 0;
// bool keyboard_ctrlmode_sw = 1;

// // Function that reads the inputs to the system and makes convertions
// void readInput() {

//     in.angle = angleLowpass.update(in.angle);
//     in.angle = in.angle - angleHighpass.update(in.angle);

//     //
//     //if(in.angle>90 || in.angle<-90){                //Sanity check angle data
//     //    while(true){
//     //        digitalWrite(pin_enable_x, LOW);        //Stop motors!
//     //        digitalWrite(pin_enable_y,LOW);
//     //        Serial.println("//Insane angle data");
//     //    }
//     //}


//     // Update notch filter parameters
//     #ifdef DYNAMICNOTCHFILTER
//     angleNotchFilter.updateFrequency(wirelengthToFrequency(in.posTrolley.y,in.magnetSw));
//     #endif

//     /*
//     if (in.magnetSw) {
//         angleNotchFilter.update(in.angle);
//     } else {
//         in.angle = angleNotchFilter.update(in.angle);
//     }*/

//     in.angle = angleNotchFilter.update(in.angle);

//     // Fetches reference point from serial readout
//     #ifndef USEPATHALGO
//     getSerialReference(&Serial,&ref);
//     #endif


//     //if some date is sent, reads it and saves in state
   
//     delay(1000);
//     if(Serial.available() > 0)
//     {     
//       state = Serial.read();
//       delay(2);   
//       flag=0;
//       Serial.println(" ");
//     //   Serial.println("Checking for new input.");
//     }   

//     // if the state is 'w' the DC motor will go UP
//     if (state == 'w') 
//     {
//         if (keyboard_magnet_sw==1) 
//         {
//             keyboard_yPos = 4;
//         }
//         else
//         {
//             keyboard_yPos = 4;      //1023 = 5V,     512 = 2,5 V.
//         }
//         if(flag == 0){
//           Serial.println("GO UP!");
//           flag=1;
//         }
//     } 

//      // if the state is 's' the DC motor will go DOWN
//     if (state == 's') {
//         if (keyboard_magnet_sw==1) 
//         {
//             keyboard_yPos = -3;
//         }
//         else
//         {
//             keyboard_yPos = -3;      
//         }
//         if(flag == 0){
//           Serial.println("GO DOWN!");
//           flag=1;
//         }
//     }

//     // if the state is 'A' the DC motor will go LEFT
//     if (state == 'a') {
//         keyboard_xPos = -3;      //1023 = 5V,     512 = 2,5 V.
//         if(flag == 0){
//           Serial.println("GO LEFT!");
//           flag=1;
//         }
//     } 

//      // if the state is 'D' the DC motor will go RIGHT
//     if (state == 'd') {
//         keyboard_xPos = 3;      //1023 = 5V,     512 = 2,5 V.
//         if(flag == 0){
//           Serial.println("GO RIGHT!");
//           flag=1;
//         }
//     }

//     // if the state is 'X' the DC motor will STOP
//     if (state == 'x') {
//         keyboard_xPos = 0;      //1023 = 5V,     512 = 2,5 V.
//         keyboard_yPos = 0; 
//         if(flag == 0){
//           Serial.println("STOP!");
//           flag=1;
//         }
//     }            

//     // if the state is 'M1' the MAGNET will turn ON
//     if (state == 'm') {
//         keyboard_magnet_sw = 1;
//         // Serial3.println("M1");
//         if(flag == 0){
//           Serial.println("MAGNET ON!");
//           flag=1;
//         }
//     }

//     // if the state is 'M0' the MAGNET will turn OFF
//     if (state == 'n') {
//         keyboard_magnet_sw = 0;
//         // Serial3.println("M0");
//         if(flag == 0){
//           Serial.println("MAGNET OFF!");
//           flag=1;
//         }
//     }

//     // if the state is 'C1' the MANUAL CONTROL will turn ON
//     if (state == 'c') {
//         keyboard_ctrlmode_sw = 1;
//         if(flag == 0){
//           Serial.println("MANUAL CONTROL ON");
//           flag=1;
//         }
//     }  
    
//     // OBS THIS IS HIGHLY UNSTABLE AND DANGEROUS. DO NOT USE!
//     // if the state is 'C0' the MANUAL CONTROL will turn OFF
//     // and the AUTOMATIC CONTROLL will turn ON
//     if (state == 'v') {
//         keyboard_ctrlmode_sw = 0;
//         if(flag == 0){
//           Serial.println("AUTOMATIC CONTROL ON");
//           flag=1;
//         }
//     }    

//     // if the state is 'p' the trolley will go to center
//     if (state == 'p') {
//         if (in.posTrolley.x < 2.19)
//         {
//             keyboard_xPos = 3;
//         }

//         if (in.posTrolley.x > 2.22)
//         {
//             keyboard_xPos = -3;
//         }

//         if (abs(in.posTrolley.x-2.20)<= 0.1)
//         {
//             keyboard_xPos = 0;
//         }

//         if(flag == 0){
//           Serial.println("GO CENTER!");
//           flag=1;
//         }
//     }  

//     //Serial.println("RAW POS: (" + String(analogRead(pin_pos_x)) + ", " + String(analogRead(pin_pos_y)) + ")");

//     in.joystick.x    = keyboard_xPos;          // Reads joystick x-direction
//     in.joystick.y    = keyboard_yPos;     // Reads joystick y-direction
//     // in.joystickSw    = digitalRead(pin_joystick_sw);        // Reads joystick switch
//     in.magnetSw      = keyboard_magnet_sw;          // Reads magnet switch on the controller
//     in.ctrlmodeSw    = keyboard_ctrlmode_sw;        // Reads control mode on the controller - digitalRead(pin_ctrlmode_sw);
//     in.posTrolley.x  = 0.0048*analogRead(pin_pos_x)-0.616; // Read x-potentiometer and convert to meters + offset
//     in.posTrolley.y  = 1.3-(0.0015*analogRead(pin_pos_y)-0.17); // Read y-potentiometer and convert to meters 
//     // in.xDriverAO1    = analogRead(pin_x_driver_AO1);        // Read analog output from driver
//     // in.xDriverAO2    = analogRead(pin_x_driver_AO2);        // Read analog output from driver
//     // in.yDriverAO1    = analogRead(pin_y_driver_AO1);        // Read analog output from driver
//     // in.yDriverAO2    = analogRead(pin_y_driver_AO2);        // Read analog output from driver
    
//     // Filter trolley position inputs
//     in.posTrolley.x = xPosLowpasss.update(in.posTrolley.x);
//     in.posTrolley.y = yPosLowpasss.update(in.posTrolley.y);

//     // Calculate container position
//     in.posContainer.x = in.posTrolley.x+(sin((in.angle*PI)/180))*in.posTrolley.y;
//     in.posContainer.y = (cos((in.angle*PI)/180))*in.posTrolley.y;

//     // Calculate speed x-axis and y-axis
//     in.velTrolley.x = xVelLowpass.update(xTrolleyVelCal.update(in.posTrolley.x));
//     in.velTrolley.y = yTrolleyVelCal.update(in.posTrolley.y);

//     // Calculate contrainer speed
//     in.velContainer.x = xContainerVelCal.update(in.posContainer.x);
//     in.velContainer.y = yContainerVelCal.update(in.posContainer.y);

//     // Calculates absolute velocity of container
//     in.velContainerAbs = sqrt(pow(in.velContainer.x,2) + pow(in.velContainer.y,2));
// }

// //Manual control
// void manualControl() {
//     // Turns on LED when in manual control
//     digitalWrite(pin_ctrlmode_led,HIGH);

//     // Calculates actuations based on joystick and trolley position
//     pwm.x = endstop(map(in.joystick.x, -10, 10, 255*0.1, 255*0.9), 0.10, 3.95, in.posTrolley.x);
//     pwm.y = endstop(map(in.joystick.y, -10, 10, 255*0.1, 255*0.9), 0.5, 1, in.posTrolley.y);

//     // Sends pwm signals to motor driver x
//     if (joystickDeadZone(in.joystick.x) == 1) {
//         analogWrite(pin_pwm_x,pwm.x);
//         digitalWrite(pin_enable_x, HIGH);
//     } else {
//         analogWrite(pin_pwm_x,127);
//         digitalWrite(pin_enable_x, LOW);
//     }

//     // Sends pwm signals to motor driver y
//     // if joystick is not in middle position
//     if (joystickDeadZone(in.joystick.y) == 1) {
//         analogWrite(pin_pwm_y,pwm.y);
//         digitalWrite(pin_enable_y, HIGH);
//     } else {
//         analogWrite(pin_pwm_y,127);
//         digitalWrite(pin_enable_y, LOW);
//     }

//     Serial.println("Millis: "+String(millis())+ ", PosTrolley X:" + String(in.posTrolley.x,3) + ",  PosTrolley Y:" + String(in.posTrolley.y,3) + ", Angle:" +String(in.angle) + ", raw pos X:" + String(analogRead(pin_pos_x)) + ", raw pos Y:" + String(analogRead(pin_pos_y)));
    
//     // + ", PWM X:" + String(pwm.x)+ "PWM Y:" + String(pwm.y));
    
//     // Turns on LED and magnet when magnet switch is active
//     if (in.magnetSw == 1) {
//         turnOnElectromagnet(true, pin_magnet_led);
//     } else {
//         turnOnElectromagnet(false, pin_magnet_led);
//     }
// }

// // Automatic control
// void automaticControl() {
//     // Turn off LED when automatic control is enabled
//     digitalWrite(pin_ctrlmode_led, LOW);

//     //Define enable value for x-axis motor
//     bool enableXmotor = true;

//     #ifdef USEPATHALGO
//     testQuayToShip.update( in.posTrolley.x, in.posTrolley.y,&ref, in.posContainer.x, in.velContainerAbs, &pathRunning, &noWireControllerOff);
//     //testShipToQuay.update( in.posTrolley.x, in.posTrolley.y,&ref, in.posContainer.x, in.velContainerAbs, &pathRunning, &InnnerLoopOn);
//     #endif

//     double xConOut=0;
    
//     // X-controller
//     if(noWireControllerOff==false){     
//         xConOut = noWireController.update(ref.x-in.posTrolley.x,Ts);
//     }
//     else{
//         double xInnerConOut = xInnerController.update(-in.angle*PI/180,Ts)*xInnerGain;
//         double xOuterConOut = xOuterController.update(ref.x-in.posTrolley.x, Ts)*xOuterGain;
//         xConOut      = xOuterConOut-xInnerConOut;
//     }
    
//     double yConOut = yController.update(ref.y-in.posTrolley.y,Ts);

//     // Make current to pwm conversion. This also removes friction in the system
//     uint8_t pwmx = currentToPwmX(xConOut, in.velTrolley.x, &enableXmotor);
//     uint8_t pwmy = currentToPwmY(yConOut, in.velTrolley.y, in.magnetSw);

//     // Definere software endstops
//     pwm.x = endstop(pwmx, 0.0, 3.75, in.posTrolley.x);  //0 er ventre og 4 højre
//     pwm.y = endstop(pwmy, 0.0, 1.25, in.posTrolley.y);   //1.25 oppe og 0.6 nede

//     // Outputs the PWM signal
//     digitalWrite(pin_enable_x, enableXmotor);
//     analogWrite(pin_pwm_x,pwm.x);

//     digitalWrite(pin_enable_y,HIGH);
//     analogWrite(pin_pwm_y,pwm.y);

//     Serial.println("Millis: "+String(millis())+ ", PosTrolley X:" + String(in.posTrolley.x,3) + ",  PosTrolley Y:" + String(in.posTrolley.y,3) + ", Angle:" + String(in.angle)+ ", ref X:"+ String(ref.x)+", Current: "+ String(xConOut)+ ",  velTrol X: "+ String(in.velTrolley.x)+",  velTrol Y: "+ String(in.velTrolley.y));

//     //  + ", PWM X:" + String(pwm.x)+ ", PWM Y:" + String(pwm.y)

//     //Serial.println("Millis: "+String(millis())+ ", PosTrolley X:" + String(in.posTrolley.x,3) + ", PosContainer X:" + String(in.posContainer.x,3) + ",  PosTrolley Y:" + String(in.posTrolley.y,3) + ", Angle:" +String(in.angle) +  ", Ref X:"+ String(ref.x,3) + ", xConOut: " + String(xConOut,3) );
//     //Serial.println(String(millis())+ ", " + String(in.posTrolley.y) + "," + String(in.velTrolley.y));
    
// }

// uint32_t start_time = 0;

// bool AutoON = false;

// // Main loop
// void loop() {

//     // Calculate loop frequency
//     uint32_t xmicros = micros();
//     loopTime = xmicros;

//     if (reader.getFloatln(&in.angle)){
//         tempAngle = in.angle;
//     }

//    readInput();
   
//     // For automatic control
//     if (in.ctrlmodeSw == 0){
//         if(AutoON == false){
            
//             xInnerController.restart();
//             xOuterController.restart();
//             yController.restart();
//             #ifdef USEPATHALGO
//             testQuayToShip.reset();
//             #endif
//             AutoON=true;
//             Serial.println("//con restarted"); 
//         }
//         automaticControl();
//         // autoJacob();
        
//         // Serial.println(String(xtime)+ ", " + String(in.posTrolley.x)+ ", " + String(in.posTrolley.y) + "," +String(in.angle) +  ","+ String(ref.x) + ", " + String(ref.y) + ", " + String(xInnerController) + ", " + String(xOuterController));

//     } else {
//         manualControl(); 
//         start_time = 0;
//         AutoON=false;
//         //Serial.println(String(millis())+ ", " + String(in.posTrolley.x,3) + ", " + String(in.posTrolley.y,3) + "," +String(in.angle) +  ","+ String(ref.x,3) + ", " + String(ref.y,3));
//         //Serial.println("Trolley: (" + String(in.posTrolley.x,3) + ", " + String(in.posTrolley.y,3) + "), " + "Angle: "+ String(in.angle) +  ", " + "Container: (" + String(in.posContainer.x,3) + ", " + String(in.posContainer.y,3) + ")");    
        
//         //Serial.println("Trolley: (" + String(in.posTrolley.x,3) + ", " + String(in.posTrolley.y,3) + "), " + "Angle: "+ String(in.angle) +  ", " + "Ref: ("+ String(ref.x,3) + ", " + String(ref.y,3) + ")");    
//     } 
//     // "Ref: ("+ String(ref.x,3) + ", " + String(ref.y,3) + ")"
//     // Serial.println(String(tempAngle)+", "+String(in.angle));
// }


// #else 

// // NotchFilter angleNotchFilter = NotchFilter(2.45,2,0.01);

// //
// // PUT SHORT TEST PROGRAMS HERE //
// //

// NotchFilter angleNotchFilter;

// void setup() {
    
//     Serial3.begin(9600);  // Communication with head
//     Serial.begin(115200); // Communication with PC
//     Serial.println("INITIALIZED");

//     angleNotchFilter = NotchFilter(2.35,2,0.01);
// }

// uint32_t timer = 0;
// float angleFloat;

// String buffer = "";

// low_pass lp = low_pass(0.02);

// fastReader reader = fastReader(&Serial3);

// void loop(){
//     reader.getFloatln(&angleFloat);
//     if (micros() >= timer + 10000) {
//         Serial.println(String(angleFloat)+", "+String(lp.update(angleNotchFilter.update(angleFloat)))+", "+String(float(micros()-timer)/1000.0));
//         timer = micros() + 10000;
//     }
// }

//  #endif