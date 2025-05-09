#include <bumperbot_controllers/twist_relay.hpp>

using std::placeholders::_1;
using namespace geometry_msgs::msg;

TwistRelay::TwistRelay(const std::string &name) : Node(name)
{
    declare_parameter("twist_in_topic", "bumperbot_controller/cmd_vel_unstamped");
    declare_parameter("twist_stamped_out_topic", "bumperbot_controller/cmd_vel");
    declare_parameter("twist_stamped_joy_in_topic", "input_joy/cmd_vel_stamped");
    declare_parameter("twist_joy_out_topic", "input_joy/cmd_vel");

    std::string twist_in_topic;
    std::string twist_stamped_out_topic;
    std::string twist_stamped_joy_in_topic;
    std::string twist_joy_out_topic;

    get_parameter("twist_in_topic", twist_in_topic);
    get_parameter("twist_stamped_out_topic", twist_stamped_out_topic);
    get_parameter("twist_stamped_joy_in_topic", twist_stamped_joy_in_topic);
    get_parameter("twist_joy_out_topic", twist_joy_out_topic);

    twist_sub_ = create_subscription<Twist>(twist_in_topic, 10, std::bind(&TwistRelay::twistCallback, this, _1));
    twist_pub_ = create_publisher<TwistStamped>(twist_stamped_out_topic, 10);
    joy_sub_ = create_subscription<TwistStamped>(twist_stamped_joy_in_topic, 10, std::bind(&TwistRelay::joyCallback, this, _1));
    joy_pub_ = create_publisher<Twist>(twist_joy_out_topic, 10);
}

void TwistRelay::twistCallback(const Twist & msg){
    TwistStamped twist_stamped;
    twist_stamped.header.stamp = get_clock()->now();
    twist_stamped.twist = msg;

    twist_pub_->publish(twist_stamped);
}

void TwistRelay::joyCallback(const TwistStamped & msg){
    Twist twist;
    twist.angular = msg.twist.angular;
    twist.linear = msg.twist.linear;

    joy_pub_->publish(twist);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistRelay>("twist_relay");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}