#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

class KalmanFilter : public rclcpp::Node
{
    public:
    KalmanFilter(const std::string & name);

    private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    void odomCallback(const nav_msgs::msg::Odometry& msg);
    void imuCallback(const sensor_msgs::msg::Imu& msg);

    void measurementUpdate();
    void statePrediction();

    double mean_;
    double variance_;
    double imu_angular_z_;
    double is_first_odom_;
    double last_angular_z_;

    double motion_;
    double motion_variance_;
    double measurement_variance_;

    nav_msgs::msg::Odometry kalman_odom_;

};
#endif //KALMAN_FILTER