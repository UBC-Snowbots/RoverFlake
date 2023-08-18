#include "Axis.h"


// Axis::Axis(int dirPin, int stepPin, int encPinA, int encPinB, long ppr, float reduction)
//   : DIR_PIN(dirPin), STEP_PIN(stepPin), ENC_PIN_A(encPinA), ENC_PIN_B(encPinB), PPR(ppr), REDUCTION(reduction) {}

void Axis::attach() {
  // Code to configure pins
   int err;
    err = gpio_pin_configure(gpio_dev, STEP_PIN, GPIO_OUTPUT);
    err = err ? err : gpio_pin_configure(gpio_dev, DIR_PIN, GPIO_OUTPUT);
   // err = err ? err : gpio_pin_configure(g)
}

void Axis::readEncoder() {
  // Code to read the encoder
}

void Axis::step(int steps) {
  // Code to move the motor steps
  steps_remaining = steps;
}

void Axis::home() {
  // Code to home the axis
}

void Axis::stepper_timer_callback(struct k_timer *timer_id) {
    static bool pulse_state = false;
    if (steps_remaining > 0) {
        if (pulse_state) {
            // End of pulse, prepare for the next one
            gpio_pin_set(gpio_dev, STEP_PIN, 0);
            steps_remaining--;    
     
           // printk("moving from %d, approaching %d\n", pos, des_pos);
  
        } else {
            // Start of pulse
            gpio_pin_set(gpio_dev, STEP_PIN, 1);
        }

        // Toggle pulse state
        pulse_state = !pulse_state;

        // Reset the timer
        k_timer_start(&stepper_timer, K_MSEC(1), K_NO_WAIT);
    } else {
        // No steps remaining, stop the timer
        k_timer_stop(&stepper_timer);
        printk("destination approached");

    }
}