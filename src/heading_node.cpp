#include "umrt-localization-ros/gps_node.hpp"

#include <iostream>

// gpsNode::gpsNode() : Node("example") {}

/**
 * Constructor:
 * Initializes node name
 * Initializes flags for message reception
 * Creates publisher for fused GPS
 * Creates subscriptions for GPS1 and GPS2
 */
GpsNode::GpsNode() : Node("gps_node"), gps1_received(false), gps2_received(false){

    rclcpp::QoS qos(10);
    qos.best_effort();

    // Publisher: creates a topic "gps/fix" to publish fused GPS messages
    // 10 = size of the message queue. If messages come too fast, store up to 10 before dropping
    fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", qos);
    
    // Subscriber for GPS1: listens to "/gps1/fix" topic
    // When a message arrives, gps1Callback() is called
    gps1_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/gps_front/fix", qos, std::bind(&GpsNode::gps1Callback, this, std::placeholders::_1));

    // Subscriber for GPS2: listens to "/gps2/fix" topic
    // When a message arrives, gps2Callback() is called
    gps2_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/gps_back/fix", qos, std::bind(&GpsNode::gps2Callback, this, std::placeholders::_1));

    // Log to console that the node has started
    RCLCPP_INFO(this->get_logger(), "Dual GPS node initialized.");
}

/**
 * Callback for GPS1
 * Stores the latest GPS1 message
 * Sets gps1_received_ to true
 * Calls processGps() to attempt fusion
 */
void GpsNode::gps1Callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    gps1_msg = *msg;
    gps1_received = true;
    processGps();
}

/**
 * Callback for GPS2
 * Stores the latest GPS2 message
 * Sets gps2_received_ to true
 * Calls processGps() to attempt fusion
 */
void GpsNode::gps2Callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    gps2_msg = *msg;
    gps2_received = true;
    processGps();
}

/**
 * Checks if both GPS messages have been received at least once
 */
bool GpsNode::gpsDataAvailable() const
{
    return gps1_received && gps2_received;
}

/**
 * Fuse GPS messages
 * Checks if both GPS messages are available
 * Averages latitude, longitude, and altitude
 * Publishes fused message
 */
void GpsNode::processGps()
{
    // Wait until both GPS messages are received
    if (!gpsDataAvailable()) return;

    sensor_msgs::msg::NavSatFix gps_msg;
    
    // Set timestamp and frame
    gps_msg.header.stamp = this->now();
    gps_msg.header.frame_id = "gps";

    // Calculate midpoint for vehicle position
    // double mid_lat = (gps1_msg.latitude + gps2_msg.latitude) / 2.0;
    // double mid_lon = (gps1_msg.longitude + gps2_msg.longitude) / 2.0;
    // double mid_alt = (gps1_msg.altitude + gps2_msg.altitude) / 2.0;

    // Rover heading direction (GPS1 back → GPS2 front)
    double dx_heading = (gps1_msg.longitude - gps2_msg.longitude) * cos(((gps2_msg.latitude + gps1_msg.latitude)/2.0) * M_PI / 180.0) * 111320.0;
    double dy_heading = (gps1_msg.latitude - gps2_msg.latitude) * 111320.0;

    double heading_angle = 0;
    std::string heading_dir = "N/A";
    if (dx_heading != 0 || dy_heading != 0) {
        heading_angle = atan2(dy_heading, dx_heading) * 180.0 / M_PI;
        if (heading_angle < 0) heading_angle += 360.0;

        if ((heading_angle >= 337.5 && heading_angle <= 360) || (heading_angle >= 0 && heading_angle < 22.5)) heading_dir = "E";
        else if (heading_angle >= 22.5 && heading_angle < 67.5) heading_dir = "NE";
        else if (heading_angle >= 67.5 && heading_angle < 112.5) heading_dir = "N";
        else if (heading_angle >= 112.5 && heading_angle < 157.5) heading_dir = "NW";
        else if (heading_angle >= 157.5 && heading_angle < 202.5) heading_dir = "W";
        else if (heading_angle >= 202.5 && heading_angle < 247.5) heading_dir = "SW";
        else if (heading_angle >= 247.5 && heading_angle < 292.5) heading_dir = "S";
        else if (heading_angle >= 292.5 && heading_angle < 337.5) heading_dir = "SE";
    }

    // Prepare NavSatFix message
    // gps_msg.latitude  = mid_lat;
    // gps_msg.longitude = mid_lon;
    // gps_msg.altitude  = mid_alt;
    gps_msg.status    = gps1_msg.status;  // copy status from GPS1

    // Publish GPS midpoint
    fix_pub_->publish(gps_msg);

    // Output
    RCLCPP_INFO(this->get_logger(), "Heading Direction: %s, Heading Angle: %.2f", heading_dir.c_str(), heading_angle);
}