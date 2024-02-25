#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>
#include <geodesy/utm.h>
#include <geometry_msgs/Point.h>

// Testing
#include "std_msgs/String.h"
#include <sstream>

// Manually add points eg. given GPS waypoint
std::vector<std::pair<double, double>> my_locations = {
    {51.4227157, -112.6411914},
    {51.4227160, -112.6411914},
    {51.4227165, -112.6411914},
    {51.4227170, -112.6411914},
    // Add more locations as needed
};

visualization_msgs::Marker gnss_points, manual_points;

// Add points once run, eg. set points for where the rover is in diff colour
// 



// Converts long, lat to UTM format
geometry_msgs::Point latLongToUTM(double latitude, double longitude) {
    geographic_msgs::GeoPoint geo_point;
    geo_point.latitude = latitude;
    geo_point.longitude = longitude;

    geometry_msgs::Point utm_point;
    geodesy::UTMPoint utm;
    geodesy::fromMsg(geo_point, utm);
    utm_point.x = utm.easting;
    utm_point.y = utm.northing;

    return utm_point;
}

void gnssCallback(const sensor_msgs::NavSatFix& msg)
{
    if (msg.status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX)
    {
        geometry_msgs::Point utm_point = latLongToUTM(msg.latitude, msg.longitude);
        gnss_points.points.push_back(utm_point);
    }
}

int main(int argc, char **argv)
{
    // Initialize node
    ros::init(argc, argv, "gnss_visualizer");
    ros::NodeHandle nh;
    
    // Grab points from nmea, publish to gnss_points 
    ros::Subscriber gnss_sub = nh.subscribe("gnss_fix", 10, gnssCallback);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker("gnss_points", 10);

    // Testing
    ros::Publisher string_pub = nh.advertise<std_msgs::String>("chatter_box",1000);
    int count = 0;

    // GNSS Points Marker
    gnss_points.header.frame_id = "/my_frame";
    gnss_points.header.stamp = ros::Time();
    gnss_points.ns = "gnss";
    gnss_points.id = 0;
    gnss_points.type = visualization_msgs::Marker::POINTS;
    gnss_points.scale.x = 0.2;
    gnss_points.scale.y = 0.2;
    gnss_points.color.g = 1.0; // Green colour
    gnss_points.color.a = 1.0;

    // Manual Points Marker
    manual_points.header.frame_id = "/my_frame"
    manual_points.header.stamp = ros::Time();
    manual_points.ns = "gnss"; // could be "manual" "Namespace to place this object in.. used in conjuction"
    manual_points.id = 1;
    manual_points.type = visualization_msgs::Marker::POINTS;
    manual_points.scale.x = 0.2;
    manual_points.scale.y = 0.2;
    manual_points.color.r = 1.0; // Red colour
    manual_points.color.a = 1.0;

    // Publish manually defined locations
    for (const auto& location : my_locations)
    {
        manual_points.points.push_back(latLongToUTM(location.first, location.second));
    }

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        // More Testing
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world" << count;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
    
        chatter_box.publish(msg);

        marker_pub.publish(gnss_points);
        marker_pub.publish(manual_points);
        ros::spinOnce();
        loop_rate.sleep();

        ++count;
    }

    return 0;
}
