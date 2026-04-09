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

    // Publisher: creates a topic "gps/fix" to publish fused GPS messages
    // 10 = size of the message queue. If messages come too fast, store up to 10 before dropping
    fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);    
    
    // Subscriber for GPS1: listens to "/gps1/fix" topic
    // When a message arrives, gps1Callback() is called
    gps1_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/gps1/fix", 10, std::bind(&GpsNode::gps1Callback, this, std::placeholders::_1));

    // Subscriber for GPS2: listens to "/gps2/fix" topic
    // When a message arrives, gps2Callback() is called
    gps2_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/gps2/fix", 10,std::bind(&GpsNode::gps2Callback, this, std::placeholders::_1));

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
    double mid_lat = (gps1_msg.latitude + gps2_msg.latitude) / 2.0;
    double mid_lon = (gps1_msg.longitude + gps2_msg.longitude) / 2.0;
    double mid_alt = (gps1_msg.altitude + gps2_msg.altitude) / 2.0;

    // Rover facing direction (GPS1 back → GPS2 front)
    double dx_facing = (gps2_msg.longitude - gps1_msg.longitude) * cos(((gps2_msg.latitude + gps1_msg.latitude)/2.0) * M_PI / 180.0) * 111320.0;
    double dy_facing = (gps2_msg.latitude - gps1_msg.latitude) * 111320.0;

    std::string facing_dir = "N/A";
    if (dx_facing != 0 || dy_facing != 0) {
        double angle = atan2(dy_facing, dx_facing) * 180.0 / M_PI;
        if (angle < 0) angle += 360.0;

        if ((angle >= 337.5 && angle <= 360) || (angle >= 0 && angle < 22.5)) facing_dir = "E";
        else if (angle >= 22.5 && angle < 67.5) facing_dir = "NE";
        else if (angle >= 67.5 && angle < 112.5) facing_dir = "N";
        else if (angle >= 112.5 && angle < 157.5) facing_dir = "NW";
        else if (angle >= 157.5 && angle < 202.5) facing_dir = "W";
        else if (angle >= 202.5 && angle < 247.5) facing_dir = "SW";
        else if (angle >= 247.5 && angle < 292.5) facing_dir = "S";
        else if (angle >= 292.5 && angle < 337.5) facing_dir = "SE";
    }

    std::string movement_dir = "N/A";
    if (prev_gps_valid) {
        // Convert latitude/longitude to radians for heading calculation
        // GPS coordinates are usually in degrees, but math functions like cos() and atan2() use radians.
        double prev_lat_rad = prev_gps_msg.latitude * M_PI / 180.0;
        double curr_lat_rad = mid_lat * M_PI / 180.0;

        /**
         * Calculate differences in meters. Calculate how far apart the two GPS units are in the east (dx) and north (dy) directions.
         * 1. dx is the East-West difference: 
         * Multiply the difference with cos to to convert longitude difference into actual east-west meters, 
         * accounting for the fact that the Earth is round and longitude lines get closer together near the poles.
         * Multiply by 111320 to convert degrees to meters
         * 
         * 2. dy is the north-south difference:
         * Calculate the difference and multiply by 111320 to convert degrees to meters
         * 
         * 3. Finally, calculate the difference in meters between previous and current midpoints
         */
        double dx_move = (mid_lon - prev_gps_msg.longitude) * cos((prev_lat_rad + curr_lat_rad)/2.0) * 111320.0;
        double dy_move = (mid_lat - prev_gps_msg.latitude) * 111320.0;

        /**
         * Returns the angle (in radians) from the x-axis (east) to the y-axis (north)
         * For example, if GPS1 is directly in front of GPS2, then the angle is 90 deg, east is 0 deg, west is 180 deg, south is 270 deg. 
         * If angle is negative (e.g., -90), add 360° → becomes 270. This is South.
         */
        // Movement angle in degrees (0 = East, 90 = North)
        if (dx_move != 0 || dy_move != 0) {
            double angle = atan2(dy_move, dx_move) * 180.0 / M_PI;
            if (angle < 0) angle += 360.0;

            if ((angle >= 337.5 && angle <= 360) || (angle >= 0 && angle < 22.5)) movement_dir = "E";
            else if (angle >= 22.5 && angle < 67.5) movement_dir = "NE";
            else if (angle >= 67.5 && angle < 112.5) movement_dir = "N";
            else if (angle >= 112.5 && angle < 157.5) movement_dir = "NW";
            else if (angle >= 157.5 && angle < 202.5) movement_dir = "W";
            else if (angle >= 202.5 && angle < 247.5) movement_dir = "SW";
            else if (angle >= 247.5 && angle < 292.5) movement_dir = "S";
            else if (angle >= 292.5 && angle < 337.5) movement_dir = "SE";
        }
    }

    // Update previous Position
    prev_gps_msg.latitude  = mid_lat;
    prev_gps_msg.longitude = mid_lon;
    prev_gps_msg.altitude  = mid_alt;
    prev_gps_valid = true;

    // Prepare NavSatFix message
    gps_msg.latitude  = mid_lat;
    gps_msg.longitude = mid_lon;
    gps_msg.altitude  = mid_alt;
    gps_msg.status    = gps1_msg.status;  // copy status from GPS1

    // Publish GPS midpoint
    fix_pub_->publish(gps_msg);

    // --- Log both heading and movement ---
    RCLCPP_INFO(this->get_logger(), "Lat: %.7f, Lon: %.7f, Alt: %.2f, Facing: %s, Moving: %s", mid_lat, mid_lon, mid_alt, facing_dir.c_str(), movement_dir.c_str());
}