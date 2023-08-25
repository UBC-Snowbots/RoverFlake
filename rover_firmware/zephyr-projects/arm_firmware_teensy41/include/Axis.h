#pragma once
#include <armFirmware.h>
// #define INIT_AXIS(i) Axis(stepPins[i], dirPins[i], encPinA[i], encPinB[i], ppr[i], red[i])


struct Axis {
   // Axis(int dirPin, int stepPin, int encPinA, int encPinB, long ppr, float reduction);
    //setup pins
    int index;
    int LIMIT_PIN[2];
    int DIR_PIN[2];
    int STEP_PIN[2];
    int ENC_PIN_A;
    int ENC_PIN_B;
    //motor gear reduction and driver PPR (make sure to factor in microstep settings)
    long PPR;
    float REDUCTION;
    bool pulse_state;
    bool moving;
    
    int max_speed;
    int home_speed;
    int current_speed;

    int macrostep;

    bool dir;
    //bool dir_signal;
    int home_dir;
    bool homing;
    bool homed = false;
    int preset_step_pos[1];

    long steps_remaining;
    long step_pos; // not from encoders
    long step_des_pos;
    
    float angle;
    struct k_timer stepper_timer;
    struct gpio_callback limit_switch_cb_data;

//   // Code to configure pins
//   const struct device *stepdev;
//   const struct device *dirdev;
//   const struct device *limdev;
//   static struct gpio_callback limit_switch_cb_data;


    void attach();
    void readEncoder();

    //void step(int steps);
    void home();

 
    
};