# Motor Control CIRC

This package contains code to run our drive motors. Our drive motors are phidgets BLDC motors that can get up to 24 V. We power them off of a 4s lipo, so ~15V.

We can bump up to 24V (6s lipo) if we need more speeeed, but at 4s our rover is already fast enough.

These motors have a high torque, and our balloon tires will mold to the ground, offering very good traction. Ashley, or someone on chassis can better explain the drive train. 

We have 6 motors, 6 drivers, and one phidgets hub that lets us use the phidgets API directly from C++ (no need for our own serial driver).

As of Oct 11 2023, we are not using ROS Control, but we have our own custom driver.

We drive the chassis like a tank, like a differential drive. We can go forwards, backwards, and turn on the spot.

Would be nice to use ROS Control, but I don't think this is necessary, and might just overcomplicate things compared to writing our driver + hardware interface from scratch.

I think a good redesign is:

(within this package)

- A node that takes in a custom message with 6 float values, and directly writes those values to the motors. (our hardware interface)

- A driver node that takes in a Twist from /cmd_vel, and then outputs the custom message to the hardware interface node.
