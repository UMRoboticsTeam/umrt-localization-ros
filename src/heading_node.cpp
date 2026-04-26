#include "umrt-localization-ros/gps_node.hpp"

#include <iostream>

// gpsNode::gpsNode() : Node("example") {}

/**
 * Constructor:
 * Initializes node name
 * Initializes flags for message reception
 * Creates publisher for fused GPS
 * Creates subscriptions for gps left and right 
 */
GpsNode::GpsNode() : Node("gps_node"), gpsLeft_active(false), gpsRight_active(false){

    rclcpp::QoS qos(10);
    qos.best_effort();

    // Publisher: creates a topic "gps/fix" to publish fused GPS messages
    // 10 = size of the message queue. If messages come too fast, store up to 10 before dropping
    fix_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", qos);
    
    // Subscriber for gpsLeft: listens to "/gps_left/fix" topic
    // When a message arrives, gpsLeftLatestMsg() is called
    gpsLeft_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>("/gps_left/fix", qos, std::bind(&GpsNode::gpsLeftLatestMsg, this, std::placeholders::_1));

    // Subscriber for gpsRight: listens to "/gps_right/fix" topic
    // When a message arrives, gpsRightLatestMsg() is called
    gpsRight_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>("/gps_right/fix", qos, std::bind(&GpsNode::gpsRightLatestMsg, this, std::placeholders::_1));

    // Log to console that the node has started
    RCLCPP_INFO(this->get_logger(), "Dual GPS node initialized.");
}

/**
 * Stores the latest message for GPS on the left
 * Sets gpsLeft_active to true
 * Calls processGps() to attempt fusion
 */
void GpsNode::gpsLeftLatestMsg(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    gpsLeft_msg = *msg;
    gpsLeft_active = true;
    processGps();
}

/**
 * Stores the latest message for GPS on the right
 * Sets gpsRight_active to true
 * Calls processGps() to attempt fusion
 */
void GpsNode::gpsRightLatestMsg(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    gpsRight_msg = *msg;
    gpsRight_active = true;
    processGps();
}

/**
 * Checks if both GPS messages have been received at least once
 */
bool GpsNode::gpsDataAvailable() const
{
    return gpsLeft_active && gpsRight_active;
}

/**
 * Fuse GPS messages
 * Checks if both GPS messages are available
 * Get the rover heading in degrees and compass direction
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

    // Calculate difference between left and right GPS in meters
    // dx = east-west movement, dy = north-south movement
    // This gives the sideways direction of the rover
    double dx = (gpsRight_msg.longitude - gpsLeft_msg.longitude) * cos(((gpsRight_msg.latitude + gpsLeft_msg.latitude) / 2.0) * M_PI / 180.0) * 111320.0;
    double dy = (gpsRight_msg.latitude - gpsLeft_msg.latitude) * 111320.0;
    double heading_angle = 0.0;
    std::string heading_dir = "N/A";
    if (dx != 0 || dy != 0)
    {
        // atan2 gives angle where 0° is East
        // Subtract 90° to convert it so 0° is North (just like a compass)
        heading_angle = atan2(dy, dx) * 180.0 / M_PI;
        heading_angle -= 90.0;

        if (heading_angle < 0)
            heading_angle += 360.0;

        if ((heading_angle >= 337.5 || heading_angle < 22.5)) heading_dir = "N";
        else if (heading_angle < 67.5) heading_dir = "NE";
        else if (heading_angle < 112.5) heading_dir = "E";
        else if (heading_angle < 157.5) heading_dir = "SE";
        else if (heading_angle < 202.5) heading_dir = "S";
        else if (heading_angle < 247.5) heading_dir = "SW";
        else if (heading_angle < 292.5) heading_dir = "W";
        else if (heading_angle < 337.5) heading_dir = "NW";
    }

    // Prepare NavSatFix message
    gps_msg.status    = gpsLeft_msg.status;  // copy status from the left GPS

    // Publish GPS midpoint
    fix_pub->publish(gps_msg);

    // Output
    RCLCPP_INFO(this->get_logger(), "Heading Direction: %s, Heading Angle: %.2f", heading_dir.c_str(), heading_angle);
}