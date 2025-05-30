#include <bumperbot_controllers/noisy_controller.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <random>

using std::placeholders::_1;

NoisyController::NoisyController(const std::string &name) : Node(name), 
    left_wheel_previous_pos_(0.0), right_wheel_previous_pos_(0.0),
    x_(0.0), y_(0.0), theta_(0.0)
{
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.17);

    get_parameter("wheel_radius", wheel_radius_);
    get_parameter("wheel_separation", wheel_separation_);

    RCLCPP_INFO(this->get_logger(), "Wheel radius: %f", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "Wheel separation: %f", wheel_separation_);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&NoisyController::jointStateCallback, this, std::placeholders::_1));
    
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/bumperbot_controller/odom_noisy", 10
    );

    previous_time_ = this->now();

    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint_ekf";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint_noisy";
}

void NoisyController::jointStateCallback(const sensor_msgs::msg::JointState &msg)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine noise_generator(seed);
    std::normal_distribution<double> left_encoder_noise(0.0, 0.005);
    std::normal_distribution<double> right_encoder_noise(0.0, 0.005);
    
    double wheel_encoder_left = msg.position.at(1) + left_encoder_noise(noise_generator);
    double wheel_encoder_right = msg.position.at(0) + right_encoder_noise(noise_generator);

    double dp_left = wheel_encoder_left - left_wheel_previous_pos_;
    double dp_right = wheel_encoder_right - right_wheel_previous_pos_;

    rclcpp::Time msg_time = msg.header.stamp;
    rclcpp::Duration dt = msg_time - previous_time_;

    previous_time_ = msg_time;
    left_wheel_previous_pos_ = wheel_encoder_left;
    right_wheel_previous_pos_ = wheel_encoder_right;

    double fi_left = dp_left / dt.seconds();
    double fi_right = dp_right / dt.seconds();
    
    double linear = (wheel_radius_ * (fi_left + fi_right)) / 2.0;
    double angular = (wheel_radius_ * (fi_right - fi_left)) / wheel_separation_;

    double d_s = (wheel_radius_ * (dp_left + dp_right)) / 2.0;
    double d_theta = (wheel_radius_ * (dp_right - dp_left)) / wheel_separation_;

    theta_ += d_theta;
    x_ += d_s * cos(theta_);
    y_ += d_s * sin(theta_);

    odom_msg_.header.stamp = get_clock()->now();
    tf2::Quaternion q;
    q.setRPY(0,0,theta_);
    odom_msg_.pose.pose.orientation.x = q.getX();
    odom_msg_.pose.pose.orientation.y = q.getY();
    odom_msg_.pose.pose.orientation.z = q.getZ();
    odom_msg_.pose.pose.orientation.w = q.getW();
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
    auto node = std::make_shared<NoisyController>("noisy_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}