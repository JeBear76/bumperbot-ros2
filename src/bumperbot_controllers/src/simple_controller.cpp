#include <bumperbot_controllers/simple_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Geometry>

using std::placeholders::_1;

SimpleController::SimpleController(const std::string &name) : Node(name)
{
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.17);

    get_parameter("wheel_radius", wheel_radius_);
    get_parameter("wheel_separation", wheel_separation_);

    RCLCPP_INFO(this->get_logger(), "Wheel radius: %f", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "Wheel separation: %f", wheel_separation_);

    wheel_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("simple_velocity_controller/commands", 10);

    velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "bumperbot_controller/cmd_vel", 10, std::bind(&SimpleController::velcocityCallback, this, std::placeholders::_1));

    // Initialize the speed conversion matrix
    speed_conversion_matrix_ << wheel_radius_ / 2.0,
        wheel_radius_ / 2.0,
        wheel_radius_ / wheel_separation_,
        -wheel_radius_ / wheel_separation_;

    RCLCPP_INFO_STREAM(this->get_logger(), "Conversion Matrix: \n" << speed_conversion_matrix_);
}

void SimpleController::velcocityCallback(const geometry_msgs::msg::TwistStamped &msg)
{
    // Convert the velocity command to wheel speeds
    Eigen::Vector2d robot_speed ( msg.twist.linear.x, msg.twist.angular.z);
    Eigen::Vector2d wheel_speed = speed_conversion_matrix_.inverse() * robot_speed;        

    // Create a message to publish the wheel speeds
    std_msgs::msg::Float64MultiArray wheel_cmd_msg;
    wheel_cmd_msg.data.push_back(wheel_speed.coeff(1)); // Left wheel speed
    wheel_cmd_msg.data.push_back(wheel_speed.coeff(0)); // Right wheel speed

    // Publish the wheel speeds
    wheel_cmd_pub_->publish(wheel_cmd_msg);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}