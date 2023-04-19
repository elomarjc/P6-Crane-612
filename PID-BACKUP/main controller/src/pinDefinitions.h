#ifndef pinDefinitions_h
#define pinDefinitions_h

// Define input pins
// #define pin_joystick_x    A9  // Joystick x-axis
// #define pin_joystick_y    A8  // Joystick y-axis
// #define pin_joystick_sw   A10 // Joystick switch
// #define pin_magnet_sw     2   // Switch magnet on / of
// #define pin_ctrlmode_sw   3   // Switch between control modes
// #define pin_x_driver_AO1  A5  // Input from x motor driver AO1
// #define pin_x_driver_AO2  A4  // Input from x motor driver AO2
// #define pin_y_driver_AO1  A3  // Input from y motor driver AO1
// #define pin_y_driver_AO2  A2  // Input from y motor driver AO2

#define pin_pos_x         A0  // 20 Input from x-axis potentiometer
#define pin_pos_y         A1  // 8 Input from y-axis potentiometer

// Define output pins
#define pin_enable_x      8   // 7 Enable driver x
#define pin_enable_y      9   // 12 Enable driver y
#define pin_pwm_x         10  // 17 PWM pin driver x
#define pin_pwm_y         11  // 19 PWM pin driver y


#define pin_magnet_led    50  // LED status of magnet
#define pin_ctrlmode_led  52  // LED status of control mode

#endif

/*
// JACOB PINS - 2023 
// Define input pins
#define pin_pos_x         A7  // Input from x-axis potentiometer
#define pin_pos_y         A8  // Input from y-axis potentiometer
#define pin_x_driver_AO1  A12  // Input from x motor driver AO1
#define pin_x_driver_AO2  A11  // Input from x motor driver AO2
#define pin_y_driver_AO1  A10  // Input from y motor driver AO1
#define pin_y_driver_AO2  A9  // Input from y motor driver AO2

// Define output pins
#define pin_enable_x      8   // Enable driver x
#define pin_enable_y      9   // Enable driver y
#define pin_pwm_x         41  // PWM pin driver x
#define pin_pwm_y         42  // PWM pin driver x




*/