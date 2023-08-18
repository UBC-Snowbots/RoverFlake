#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>
#include <geodesy/utm.h>
#include <geometry_msgs/Point.h>

// Manually add points eg. given GPS waypoint
std::vector<std::pair<double, double>> my_locations = {
    {51.4227157, -112.6411914},
    {51.4227160, -112.6411914},
    {51.4227165, -112.6411914},
    {51.4227170, -112.6411914},
    // Add more locations as needed
};


visualization_msgs::Marker gnss_points;
visualization_msgs::Marker manual_points;

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
    ros::init(argc, argv, "gnss_visualizer");
    ros::NodeHandle nh;

    ros::Subscriber gnss_sub = nh.subscribe("gnss_fix", 10, gnssCallback);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("gnss_points", 10);

    // GNSS Points Marker
    gnss_points.header.frame_id = "map";
    gnss_points.ns = "gnss";
    gnss_points.id = 0;
    gnss_points.type = visualization_msgs::Marker::POINTS;
    gnss_points.scale.x = 999;
    gnss_points.scale.y = 999;
    gnss_points.color.g = 1.0; // Green color
    gnss_points.color.a = 1.0;

    // Manual Points Marker
    manual_points.header.frame_id = "map";
    manual_points.ns = "manual";
    manual_points.id = 1;
    manual_points.type = visualization_msgs::Marker::POINTS;
    manual_points.scale.x = 999;
    manual_points.scale.y = 999;
    manual_points.color.r = 1.0; // Red color
    manual_points.color.a = 1.0;

    // Add manually defined locations
    for (const auto& location : my_locations)
    {
        manual_points.points.push_back(latLongToUTM(location.first, location.second));
    }

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        marker_pub.publish(gnss_points);
        marker_pub.publish(manual_points);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
