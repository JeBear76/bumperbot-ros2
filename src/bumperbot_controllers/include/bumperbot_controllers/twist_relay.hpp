#ifndef TWIST_RELAY_HPP
#define TWIST_RELAY_HPP
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>


class TwistRelay : public rclcpp::Node
{
public:
    TwistRelay(const std::string &name);

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr joy_pub_;
    void twistCallback(const geometry_msgs::msg::Twist &msg);
    void joyCallback(const geometry_msgs::msg::TwistStamped &msg);

};
#endif // TWIST_RELAY_HPP