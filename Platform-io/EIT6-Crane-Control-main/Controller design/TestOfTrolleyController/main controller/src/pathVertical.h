#include <Arduino.h>
#include "dataStructures.h"

class QauyToShipV{
    public:
        QauyToShipV(int electroMagnetLED);
        int update(float xPos, float yPos, xy_float  *ref , float xContainer, float containerSpeed, bool *pathRunning, bool *InnnerLoopOn);
        void reset();

    private:
        int step=0;
        uint16_t failTime =0;
        float *LxRefpoint=0;
        float *LyRefpoint=0;
        int LelectroMagnetLED=0;
};



class ShipToQauyV{
    public:
        ShipToQauyV(int electroMagnetLED);
        void update(float xPos, float yPos, xy_float *ref, float xContainer, float containerSpeed,  bool *pathRunning, bool *InnnerLoopOn);
        void reset();

    private:
        int step=0;
        uint16_t failTime =0;
        uint16_t failtimm1=0;
        float *LxRefpoint=0;
        float *LyRefpoint=0;
        int LelectroMagnetLED=0;
};