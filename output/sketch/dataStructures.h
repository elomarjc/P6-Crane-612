#line 1 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\Simple_gantry_code\\dataStructures.h"

// Data struct definitions

#ifndef dataStructures_h
#define dataStructures_h

struct xy_float {
    float x = 0;
    float y = 0;
};

struct xy_pwm {
    int x = 127;
    int y = 127;
};

struct inputs {
    bool joystickSw, magnetSw, ctrlmodeSw = 1;
    xy_float joystick,posTrolley,velTrolley,posContainer,velContainer;
    float angle,velContainerAbs;
    int xDriverAO1, xDriverAO2, yDriverAO1, yDriverAO2;
};

#endif