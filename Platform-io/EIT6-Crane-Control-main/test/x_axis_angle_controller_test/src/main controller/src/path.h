#include <Arduino.h>
#include "dataStructures.h"

class QauyToShip{
    public:
        QauyToShip(int electroMagnetLED);
        void update(float xPos, float yPos, xy_float  *ref , float xContainer, float containerSpeed, bool *pathRunning);
        void reset();

    private:
        int step=0;
        uint16_t failTime =0;
        float *LxRefpoint=0;
        float *LyRefpoint=0;
        int LelectroMagnetLED=0;
};



class ShipToQauy{
    public:
        ShipToQauy(int electroMagnetLED);
        void update(float xPos, float yPos, xy_float *ref, float xContainer, float containerSpeed);
        void reset();

    private:
        int step=0;
        uint16_t failTime =0;
        uint16_t failtimm1=0;
        float *LxRefpoint=0;
        float *LyRefpoint=0;
        int LelectroMagnetLED=0;
};