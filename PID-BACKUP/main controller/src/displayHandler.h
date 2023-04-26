#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>

void initializeDisplay(Adafruit_SSD1306 *display);
void displayInfo(Adafruit_SSD1306 *display,inputs in, float frequency,uint32_t *timer);