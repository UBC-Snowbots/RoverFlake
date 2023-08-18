#pragma once
#include <armFirmware.h>
// #define INIT_AXIS(i) Axis(stepPins[i], dirPins[i], encPinA[i], encPinB[i], ppr[i], red[i])


struct Axis {
   // Axis(int dirPin, int stepPin, int encPinA, int encPinB, long ppr, float reduction);
    
    int DIR_PIN;
    int STEP_PIN;
    int ENC_PIN_A;
    int ENC_PIN_B;
    long PPR;
    float REDUCTION;
    long steps_remaining;
    static struct k_timer stepper_timer;


    void attach();
    void readEncoder();
    void step(int steps);
    void home();
    void stepper_timer_callback(struct k_timer *timer_id);
 
    
};