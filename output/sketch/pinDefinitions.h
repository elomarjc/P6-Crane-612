#line 1 "C:\\Users\\andre\\OneDrive - Aalborg Universitet\\_Universitet\\EIT6\\_P6\\Github\\P6-Crane-612\\P-controller\\pinDefinitions.h"
#ifndef pinDefinitions_h
#define pinDefinitions_h

// Define input pins
#define pin_pos_x         A0  // Input from x-axis potentiometer
#define pin_pos_y         A1  // Input from y-axis potentiometer
#define pin_x_driver_AO1  A5  // Input from x motor driver AO1
#define pin_x_driver_AO2  A4  // Input from x motor driver AO2
#define pin_y_driver_AO1  A3  // Input from y motor driver AO1
#define pin_y_driver_AO2  A2  // Input from y motor driver AO2

// Define output pins
#define pin_enable_x      8   // Enable driver x
#define pin_enable_y      9   // Enable driver y
#define pin_pwm_x         10  // PWM pin driver x
#define pin_pwm_y         11  // PWM pin driver y

#endif