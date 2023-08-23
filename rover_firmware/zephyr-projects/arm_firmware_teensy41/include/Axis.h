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
    int speed;
    int macrostep;

    bool dir;

    long steps_remaining;
    long step_pos; // not from encoders
    long step_des_pos;
    
    float angle;
    struct k_timer stepper_timer;


    void attach();
    void readEncoder();
    //void step(int steps);
    void home();
 
    
};