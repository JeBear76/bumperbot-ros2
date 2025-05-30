#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

class turtlesim_kinematics : public rclcpp::Node
{
private:
    // Define a subscription to the turtlesim pose topic
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_subscription_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_pose_subscription_;
    turtlesim::msg::Pose turtle1_last_pose_;
    turtlesim::msg::Pose turtle2_last_pose_;

public:
    turtlesim_kinematics() : rclcpp::Node("turtlesim_kinematics")
    {
        // Create a subscription to the turtlesim pose topic
        RCLCPP_INFO(this->get_logger(), "turtlesim_kinematics node has been started");
        turtle1_pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10,
            [this](const turtlesim::msg::Pose & pose)
            {
                turtle1_last_pose_ = pose;
            });

        turtle2_pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle2/pose", 10,
            [this](const turtlesim::msg::Pose & pose)
            {
                turtle2_last_pose_ = pose;
                // Calculate the distance between the two turtles
                
                auto translation_x = turtle2_last_pose_.x - turtle1_last_pose_.x;
                auto translation_y = turtle2_last_pose_.y - turtle1_last_pose_.y;
                auto theta = turtle2_last_pose_.theta - turtle1_last_pose_.theta;
                auto theta_degrees = theta * 180.0 / M_PI;

                RCLCPP_INFO(this->get_logger(), "Turtle translation:\n\t\
                Tx: %f\n\t\
                Ty: %f\n\
                Theta: %f degrees\n\
                Rotation Matrix:\n\t\
                |cos(Ttheta) -sin(Ttheta)|:| %f\t%f|\n\t\
                |sin(Ttheta)  cos(Ttheta)|:| %f\t%f|\n",
                        translation_x, 
                        translation_y, 
                        theta_degrees,
                        cos(theta),
                        -sin(theta),
                        sin(theta),
                        cos(theta));
            });
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtlesim_kinematics>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}