#ifndef SIMPLE_TF_KINEMATICS
#define SIMPLE_TF_KINEMATICS
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <bumperbot_msgs/srv/get_transform.hpp>

class SimpleTfKinematics : public rclcpp::Node
{
public:
    SimpleTfKinematics(const std::string &node_name = "simple_tf_kinematics");

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped static_transform_stamped_;
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped_;
    rclcpp::TimerBase::SharedPtr dynamic_tf_timer_;

    rclcpp::Service<bumperbot_msgs::srv::GetTransform>::SharedPtr get_transform_service_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    
    void publishDynamicTransform();
    bool getTransformCallback(
        const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Request> request,
        const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Response> response);

    double x_increment_;
    double last_x_;
};
#endif // SIMPLE_TF_KINEMATICS