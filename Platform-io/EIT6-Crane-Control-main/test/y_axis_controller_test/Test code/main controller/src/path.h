#include <Arduino.h>


class QauyToShip{
    public:
        QauyToShip();
        void QauyToShip::update(float xPos, float yPos, float xContainer, float containerSpeed, float *xRefpoint, float *yRefpoint, int electroMagnetLED);
        void QauyToShip::reset();

    private:
        int step=0;
        uint16_t failTime =0;
};



class ShipToQauy{
    public:
        void ShipToQauy::update(float xPos, float yPos, float xContainer, float containerSpeed, float *xRefpoint, float *yRefpoint, int electroMagnetLED);
        void ShipToQauy::reset();

    private:
        int step=0;
        uint16_t failTime =0;
};