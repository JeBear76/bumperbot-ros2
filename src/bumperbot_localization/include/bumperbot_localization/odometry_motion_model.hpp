#ifndef ODOMETRY_MOTION_MODEL_HPP
#define ODOMETRY_MOTION_MODEL_HPP
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

class OdometryMotionModel : public rclcpp::Node
{
    public:
    OdometryMotionModel(const std::string & name);

    private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
    void odomCallback(const nav_msgs::msg::Odometry& msg);

    double is_first_odom;
    double last_odom_x;
    double last_odom_y;
    double last_odom_theta;
    
    double alpha1;
    double alpha2;
    double alpha3;
    double alpha4;
    int nr_samples;

    geometry_msgs::msg::PoseArray samples;

};
#endif //ODOMETRY_MOTION_MODEL