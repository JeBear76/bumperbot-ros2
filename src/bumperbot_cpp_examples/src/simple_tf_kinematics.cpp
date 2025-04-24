#include <bumperbot_cpp_examples/simple_tf_kinematics.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

SimpleTfKinematics::SimpleTfKinematics(const std::string &node_name) : Node(node_name), x_increment_(0.01), last_x_(0.0)
{
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    get_transform_service_ = create_service<bumperbot_msgs::srv::GetTransform>(
        "get_transform",
        std::bind(&SimpleTfKinematics::getTransformCallback, this, _1, _2));

    static_transform_stamped_.header.frame_id = "bumperbot_base";
    static_transform_stamped_.child_frame_id = "bumperbot_top";        
    static_transform_stamped_.header.stamp = get_clock()->now();
    static_transform_stamped_.transform.translation.x = 0.0;
    static_transform_stamped_.transform.translation.y = 0.0;
    static_transform_stamped_.transform.translation.z = 0.3;
    static_transform_stamped_.transform.rotation.x = 0.0;
    static_transform_stamped_.transform.rotation.y = 0.0;
    static_transform_stamped_.transform.rotation.z = 0.0;
    static_transform_stamped_.transform.rotation.w = 1.0;
    
    static_tf_broadcaster_->sendTransform(static_transform_stamped_);

    RCLCPP_INFO(this->get_logger(), "Static transform from base_link to base_footprint published.");

    dynamic_tf_timer_ = create_wall_timer(
        0.1s,
        std::bind(&SimpleTfKinematics::publishDynamicTransform, this));
}

void SimpleTfKinematics::publishDynamicTransform()
{
    dynamic_transform_stamped_.header.frame_id = "odom";
    dynamic_transform_stamped_.child_frame_id = "bumperbot_base";
    dynamic_transform_stamped_.header.stamp = get_clock()->now();
    dynamic_transform_stamped_.transform.translation.x = last_x_ + x_increment_;
    dynamic_transform_stamped_.transform.translation.y = 0.0;
    dynamic_transform_stamped_.transform.translation.z = 0.0;
    dynamic_transform_stamped_.transform.rotation.x = 0.0;
    dynamic_transform_stamped_.transform.rotation.y = 0.0;
    dynamic_transform_stamped_.transform.rotation.z = 0.0;
    dynamic_transform_stamped_.transform.rotation.w = 1.0;

    dynamic_tf_broadcaster_->sendTransform(dynamic_transform_stamped_);
    
    last_x_ += x_increment_;
}

bool SimpleTfKinematics::getTransformCallback(
    const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Request> request,
    const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Response> response)
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("SimpleTfKinematics"), "Received request for transform from " << request->frame_id.c_str() << " to " << request->child_frame_id.c_str());
    
    try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(request->frame_id, request->child_frame_id, tf2::TimePointZero);
        response->transform = transform;
        response->success = true;
        return true;
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("SimpleTfKinematics"), "Could not get transform: from " << request->frame_id.c_str() << " to " << request->child_frame_id.c_str() << " : " << ex.what());
        return false;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTfKinematics>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}