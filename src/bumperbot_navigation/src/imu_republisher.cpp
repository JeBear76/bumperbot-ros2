#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

void imuCallback(const sensor_msgs::msg::Imu &msg)
{
    sensor_msgs::msg::Imu imu;
    imu = msg;
    imu.header.frame_id = "base_footprint_ekf";
    imu_pub_->publish(imu);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("imu_republisher");
    rclcpp::sleep_for(1s);
    imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>("/imu/ekf", 10);
    auto imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>("/imu/out", 10, imuCallback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}