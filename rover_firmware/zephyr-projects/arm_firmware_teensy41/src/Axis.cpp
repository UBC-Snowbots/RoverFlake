#include "Axis.h"


// Axis::Axis(int dirPin, int stepPin, int encPinA, int encPinB, long ppr, float reduction)
//   : DIR_PIN(dirPin), STEP_PIN(stepPin), ENC_PIN_A(encPinA), ENC_PIN_B(encPinB), PPR(ppr), REDUCTION(reduction) {}

// void Axis::attach() {
//   // Code to configure pins
//    int zoo;
//     zoo = gpio_pin_configure(gpio_dev, STEP_PIN, GPIO_OUTPUT);
//     zoo = zoo ? zoo : gpio_pin_configure(gpio_dev, DIR_PIN, GPIO_OUTPUT);

//   //  // err = err ? err : gpio_pin_configure(g)
// }

void Axis::readEncoder() {
  // Code to read the encoder
}



void Axis::home() {
  // Code to home the axis
}
