/*
Created By: Tate Kolton, Graeme Dockrill
Created On: December 21, 2021
Updated On: January 10, 2023
Description: Main firmware for driving a 6 axis arm via ROS on a teensy 4.1 MCU
*/

// header file with all constants defined and libraries included
#include "armFirmware.h"

// setup function to initialize pins and provide initial homing to the arm.
void setup() {

  Serial.begin(19200);

  for (int i = 0; i < NUM_AXES; i++) {
    ENC_STEPS_PER_DEG[i] = ppr[i] * red[i] * (ENC_MULT[i] / 360.0);

    min_steps[i] = -homeCompSteps[i];
    max_steps[i] = max_steps[i] - homeCompSteps[i];
  }

  min_steps[0] = -max_steps[0];

  // initializes end effector motor
  pinMode(EEstepPin, OUTPUT);
  pinMode(EEdirPin, OUTPUT);
  endEff.setMinPulseWidth(200);
  endEff.setMaxSpeed(speedEE);
  endEff.setAcceleration(accEE);
  endEff.setCurrentPosition(1000);

  // initializes step pins, direction pins, limit switch pins, and stepper motor objects for accelStepper library
  for (i = 0; i < NUM_AXES; i++) {

    pinMode(limPins[i], INPUT_PULLUP);

     
  }

  // waits for user to press "home" button before rest of functions are available
  // waitForHome();
}

// main program loop
void loop() {

//test loops
for(int i = 0; i < 6; i++){
    Serial.print("READY TO TEST LIMIT OF AXIS: ");
    Serial.println(i + 1);
  while(digitalRead(limPins[i]) == LOW){
      //just chillin
      delay(10); //just to be extra chillin
  }
    Serial.print("LIMIT ");
    Serial.print(i + 1);
    Serial.println(" CONNECTION VERIFIED");
  }

}








