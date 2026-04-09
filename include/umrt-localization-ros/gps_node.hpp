#ifndef GPS_NODE_HPP
#define GPS_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

/**
 * This is an example node provided to demonstrate file structure and CMake configuration.
 */
class GpsNode: public rclcpp::Node {
public:
    // Constructs a GpsNode. Runs when the node starts and is used to initialize subscriptions, publishers, etc.
    GpsNode();

private:

    // Publisher: Sends messages to a topic. Other nodes can subscribe to it
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;

    // Subscriber: Listens to a topic (like /gps1/fix). Receives messages when published.
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps2_sub_;

    // Stores the latest GPS data
    sensor_msgs::msg::NavSatFix gps1_msg;
    sensor_msgs::msg::NavSatFix gps2_msg;

    // Ensures both gps are producing an output before processing
    bool gps1_received;
    bool gps2_received;
    bool prev_gps_valid;               // tracks if previous GPS is valid

    // Store previous fused midpoint
    sensor_msgs::msg::NavSatFix prev_gps_msg;  // stores previous fused GPS midpoint

    // Callbacks: Runs automatically when a message arrives
    void gps1Callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void gps2Callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    // Combines both gps readings
    void processGps();

    bool gpsDataAvailable() const;
    
};

#endif //GPS_NODE_HPP