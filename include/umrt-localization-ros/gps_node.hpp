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
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub;

    // Subscriber: Listens to a topic (like /gps_left/fix). Receives messages when published.
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsLeft_sub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsRight_sub;

    // Stores the latest GPS data
    sensor_msgs::msg::NavSatFix gpsLeft_msg;
    sensor_msgs::msg::NavSatFix gpsRight_msg;

    // Ensures both gps are producing an output before processing
    bool gpsLeft_active;
    bool gpsRight_active;

    // Stores the latest message automatically.
    void gpsLeftLatestMsg(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void gpsRightLatestMsg(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    // Process the message and give output
    void processGps();

    bool gpsDataAvailable() const;
    
};

#endif //GPS_NODE_HPP