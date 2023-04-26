#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>
#include "dataStructures.h"

#ifndef SCREEN_ADDRESS
#define SCREEN_ADDRESS  0x3C
#endif

void initializeDisplay(Adafruit_SSD1306 *display){
    // Initialize screen
    delay(2000); // This delay ensures that the screen is started before sending data to it
    if(!display->begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }
    display->clearDisplay();
    display->setTextSize(1);
    display->setTextColor(SSD1306_WHITE);
    display->setCursor(0,0);
    display->println(F("Wait crane starting.."));
    display->setCursor(0,20);
    display->println(F("EIT6 CE6 630"));
    display->display();
    delay(2000);

    // Printing starting on serial 0
    Serial.println("Starting...");

    display->clearDisplay();
    display->setTextSize(1);
    display->setTextColor(SSD1306_WHITE);
    display->setCursor(0,0);
    display->println(F("Crane Ready!"));
    display->display();
}

void displayInfo(Adafruit_SSD1306 *display,inputs in, float frequency,uint32_t *timer) {
    if (1e6/(micros()-*timer) < 5){ // Update OLED screen with xx frequency
        *timer = micros();
        display->clearDisplay();
        display->setTextSize(1);
        display->setTextColor(SSD1306_WHITE);
        display->setCursor(0,0);
        display->println(("X "+String(in.posTrolley.x)));
        display->setCursor(70,0);
        display->println(("Y "+String(in.posTrolley.y)));
        display->setCursor(0,10);
        display->println(("A "+String(in.angle)));
        display->setCursor(70,10);
        display->println(("Hz "+String(frequency)));
        display->setCursor(0,20);
        display->println(("Xc "+String(in.posContainer.x)));
        display->setCursor(70,20);
        display->println(("Yc "+String(in.posContainer.y)));
        display->display();
    }
}