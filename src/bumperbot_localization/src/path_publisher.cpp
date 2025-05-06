#include <bumperbot_localization/path_publisher.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using std::placeholders::_1;

PathPublisher::PathPublisher(const std::string &name) : Node(name){
    declare_parameter("odom_topic", "/bumperbot_controller/odom");
    declare_parameter("path_topic", "/bumperbot_controller/trajectory");

    auto odom_topic = get_parameter("odom_topic").as_string();
    auto path_topic = get_parameter("path_topic").as_string();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 
        10, 
        std::bind(&PathPublisher::odomCallback, this, _1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, 10);

    RCLCPP_INFO_STREAM(this->get_logger(), "Subscriber to " << odom_topic <<" and Publisher on " << path_topic <<" started.");
}


void PathPublisher::odomCallback(const nav_msgs::msg::Odometry &msg){
    path_msg_.header.frame_id = msg.header.frame_id;
    geometry_msgs::msg::PoseStamped current_pose_stamped;
    current_pose_stamped.header.frame_id = msg.header.frame_id;
    current_pose_stamped.header.stamp = msg.header.stamp;
    current_pose_stamped.pose = msg.pose.pose;
    path_msg_.poses.push_back(current_pose_stamped);

    path_pub_->publish(path_msg_);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPublisher>("path_publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}