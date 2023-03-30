#include <Arduino.h>


class QauyToShip{
    public:
        QauyToShip();
        void update(float xPos, float yPos, float xContainer, float containerSpeed, float *xRefpoint, float *yRefpoint, int electroMagnetLED);
        void reset();

    private:
        int step = 0;
        uint16_t failTime = 0;
};

class ShipToQauy{
    public:
        void update(float xPos, float yPos, float xContainer, float containerSpeed, float *xRefpoint, float *yRefpoint, int electroMagnetLED);
        void reset();

    private:
        int step = 0;
        uint16_t failTime = 0;
};