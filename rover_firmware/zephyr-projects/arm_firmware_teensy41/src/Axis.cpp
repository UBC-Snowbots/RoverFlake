#include "Axis.h"

//struct k_timer stepper_timer;

// Axis::Axis(int dirPin, int stepPin, int encPinA, int encPinB, long ppr, float reduction)
//   : DIR_PIN(dirPin), STEP_PIN(stepPin), ENC_PIN_A(encPinA), ENC_PIN_B(encPinB), PPR(ppr), REDUCTION(reduction) {}
  // Code to configure pins
  const struct device *stepdev;
  const struct device *dirdev;
  const struct device *limdev;


void Axis::attach() {



	switch (STEP_PIN[0])
	{
	case 1:
		stepdev = gpio1_dev;
		break;
	
	case 2:
		stepdev = gpio2_dev;

		break;
	case 4:
		stepdev = gpio2_dev;
		break;
	default:
		break;
	}

  switch (DIR_PIN[0])
	{
	case 1:
		dirdev = gpio1_dev;
		break;
	
	case 2:
		dirdev = gpio2_dev;

		break;
	case 4:
		dirdev = gpio2_dev;
		break;
	default:
		break;
	}

  switch (LIMIT_PIN[0])
	{
	case 1:
		limdev = gpio1_dev;
		break;
	
	case 2:
		limdev = gpio2_dev;

		break;
	case 4:
		limdev = gpio2_dev;
		break;
	default:
		break;
	}

    int zoo;
    zoo = gpio_pin_configure(stepdev, STEP_PIN[1], GPIO_OUTPUT);
    zoo = zoo ? zoo : gpio_pin_configure(dirdev, DIR_PIN[1], GPIO_OUTPUT);
    zoo = zoo ? zoo : gpio_pin_configure(limdev, LIMIT_PIN[1], GPIO_INPUT | GPIO_PULL_UP);
	// gpio_init_callback(&limit_switch_cb_data, limit_switch_callback, BIT(LIMIT_PIN[1]));
	// gpio_add_callback(limdev, &limit_switch_cb_data);
	// gpio_pin_interrupt_configure(limdev, LIMIT_PIN[1], GPIO_INT_EDGE_FALLING);



    //gpio_pin_configure(gpio_dev, axes[i].STEP_PIN, GPIO_OUTPUT);
	  // gpio_pin_configure(gpio_dev, axes[i].DIR_PIN, GPIO_OUTPUT);

  //  // err = err ? err : gpio_pin_configure(g)
}



void Axis::readEncoder() {
  // Code to read the encoder
}



void Axis::home() {
  // Code to home the axis
}
