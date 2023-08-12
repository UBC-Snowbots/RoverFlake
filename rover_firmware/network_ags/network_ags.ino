#include <AccelStepper.h>

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
// Since you're using the DIR/STEP interface, you can use this driver instance
#define DIR_PIN 27
#define STEP_PIN 14

AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  // Set the maximum speed and acceleration:
  stepper.setMaxSpeed(2500.0);
  stepper.setAcceleration(200.0);

  // Set the initial speed and the target position
  stepper.setSpeed(200.0);
  stepper.moveTo(20000);
}

void loop() {
  // If at the end of travel go to the other end
  if (stepper.distanceToGo() == 0) {
    stepper.moveTo(-stepper.currentPosition());
  }
  stepper.run();
}
