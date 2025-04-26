#ifndef PATH_PUBLISHER_HPP
#define PATH_PUBLISHER_HPP
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

class PathPublisher : public rclcpp::Node
{
    public:
        PathPublisher(const std::string& name);

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

        void odomCallback(const nav_msgs::msg::Odometry &msg);

        nav_msgs::msg::Path path_msg_;
};
#endif