#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_gnss_publisher");
  ros::NodeHandle nh;

  ros::Publisher gnss_pub = nh.advertise<sensor_msgs::NavSatFix>("/gnss_fix", 10);

  ros::Rate loop_rate(10); // 10 Hz, keep it consistent with our rviz publisher

  sensor_msgs::NavSatFix fake_data; // Data point

  fake_data.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  fake_data.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

  while (ros::ok())
  {
    // Set header fields
    fake_data.header.stamp = ros::Time::now();
    fake_data.header.frame_id = "/my_frame";

    // Modify these values to simulate different GNSS data points
    fake_data.latitude = 49.2827;
    fake_data.longitude = -123.1207;

    gnss_pub.publish(fake_data);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}