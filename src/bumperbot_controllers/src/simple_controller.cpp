#include <bumperbot_controllers/simple_controller.hpp>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.hpp>

using std::placeholders::_1;

SimpleController::SimpleController(const std::string &name) : Node(name), 
    left_wheel_previous_pos_(0.0), right_wheel_previous_pos_(0.0),
    x_(0.0), y_(0.0), theta_(0.0)
{
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.17);

    get_parameter("wheel_radius", wheel_radius_);
    get_parameter("wheel_separation", wheel_separation_);

    RCLCPP_INFO(this->get_logger(), "Wheel radius: %f", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "Wheel separation: %f", wheel_separation_);

    wheel_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);

    velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/bumperbot_controller/cmd_vel", 10, std::bind(&SimpleController::velocityCallback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&SimpleController::jointStateCallback, this, std::placeholders::_1));
    
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/bumperbot_controller/odom", 10
    );

    previous_time_ = this->now();

    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "bumperbot_footprint";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    // Initialize the speed conversion matrix
    speed_conversion_matrix_ << wheel_radius_ / 2.0,
        wheel_radius_ / 2.0,
        wheel_radius_ / wheel_separation_,
        -wheel_radius_ / wheel_separation_;

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint";

    RCLCPP_INFO_STREAM(this->get_logger(), "Conversion Matrix: \n" << speed_conversion_matrix_);
}

void SimpleController::velocityCallback(const geometry_msgs::msg::TwistStamped &msg)
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

void SimpleController::jointStateCallback(const sensor_msgs::msg::JointState &msg)
{
    double dp_left = msg.position.at(1) - left_wheel_previous_pos_;
    double dp_right = msg.position.at(0) - right_wheel_previous_pos_;

    rclcpp::Time msg_time = msg.header.stamp;
    rclcpp::Duration dt = msg_time - previous_time_;

    previous_time_ = msg_time;
    left_wheel_previous_pos_ = msg.position.at(1);
    right_wheel_previous_pos_ = msg.position.at(0);

    double fi_left = dp_left / dt.seconds();
    double fi_right = dp_right / dt.seconds();
    
    auto linear = (wheel_radius_ * (fi_left + fi_right)) / 2.0;
    auto angular = (wheel_radius_ * (fi_right - fi_left)) / wheel_separation_;

    double d_s = (wheel_radius_ * (dp_left + dp_right)) / 2.0;
    double d_theta = (wheel_radius_ * (dp_right - dp_left)) / wheel_separation_;

    theta_ += d_theta;
    x_ += d_s * cos(theta_);
    y_ += d_s * sin(theta_);

    // RCLCPP_INFO_STREAM(rclcpp::get_logger("SimpleController"), 
    // "\nLinear: " << linear 
    // << "\nAngular: " << angular
    // << "\nX: " << x_
    // << "\nY: " << y_
    // << "\ntheta: " << theta_);


    odom_msg_.header.stamp = get_clock()->now();
    tf2::Quaternion q;
    q.setRPY(0,0,theta_);
    odom_msg_.pose.pose.orientation.x = q.x();
    odom_msg_.pose.pose.orientation.y = q.y();
    odom_msg_.pose.pose.orientation.z = q.z();
    odom_msg_.pose.pose.orientation.w = q.w();
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.twist.twist.linear.x = linear;
    odom_msg_.twist.twist.angular.z = angular;

    odometry_pub_->publish(odom_msg_);

    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.getX();
    transform_stamped_.transform.rotation.y = q.getY();
    transform_stamped_.transform.rotation.z = q.getZ();
    transform_stamped_.transform.rotation.w = q.getW();
    transform_stamped_.header.stamp = get_clock()->now();
    transform_broadcaster_->sendTransform(transform_stamped_);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}