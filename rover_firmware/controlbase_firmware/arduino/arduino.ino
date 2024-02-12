#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

// Define the publisher for the potentiometer state
std_msgs::Int16 pot_msg;
ros::Publisher pot_publisher("potentiometer_state", &pot_msg);

const int potPin = A0;  // Analog pin connected to the potentiometer
//fdshjsfdf
void setup() {
  nh.initNode();
  nh.advertise(pot_publisher);

  pinMode(potPin, INPUT);
}

void loop() {
  // Read the analog value from the potentiometer
  int potValue = analogRead(potPin);

  // Map the analog value (0-1023) to the range 0-100
  pot_msg.data = map(potValue, 0, 1023, 0, 100);

  // Publish the potentiometer state
  pot_publisher.publish(&pot_msg);

  // Allow time for communication
  nh.spinOnce();

  // Add a delay to control the publishing frequency
  delay(100);
}
//sudo chmod 666 /dev/ttyUSB0
//rosrun rosserial_python serial_node.py /dev/ttyUSB0
