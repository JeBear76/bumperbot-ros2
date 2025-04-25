#!/usr/bin/env python3
import rclpy
import numpy as np
import math

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from tf_transformations import quaternion_from_euler

class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta = 0.0

        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_footprint"
        self.odom.pose.pose.orientation.x = 0.0
        self.odom.pose.pose.orientation.y = 0.0
        self.odom.pose.pose.orientation.z = 0.0
        self.odom.pose.pose.orientation.w = 1.0

        self.wheel_radius_ = (
            self.get_parameter("wheel_radius").get_parameter_value().double_value
        )
        self.wheel_separation_ = (
            self.get_parameter("wheel_separation").get_parameter_value().double_value
        )

        self.get_logger().info(
            f"Paramters:\n\t Wheel Radius: {self.wheel_radius_}\n\t Wheel Separation: {self.wheel_separation_}"
        )

        self.left_wheel_previous_pos_ = 0.0
        self.right_wheel_previous_pos_ = 0.0

        self.previous_time_ = self.get_clock().now()

        self.sub = self.create_subscription(
            TwistStamped, "bumperbot_controller/cmd_vel", self.velocityCallback, 10
        )

        self.joint_subscriber_ = self.create_subscription(
            JointState, "joint_states", self.jointCallback, 10
        )

        self.command_pub_ = self.create_publisher(
            Float64MultiArray, "simple_velocity_controller/commands", 10
        )

        self.odom_pub_ = self.create_publisher(
            Odometry, "bumperbot_controller/odom", 10
        )

        self.speed_conversion_ = np.array(
            [
                [self.wheel_radius_ / 2, self.wheel_radius_ / 2],
                [
                    self.wheel_radius_ / self.wheel_separation_,
                    -self.wheel_radius_ / self.wheel_separation_,
                ],
            ],
        )

        self.get_logger().info(
            f"""
            Conversion Matrix:\n\t
            {self.speed_conversion_}\n
            """
        )
        self.get_logger().info("Simple Controller Node has been started.")

    def velocityCallback(self, msg):
        robot_speed = np.array([[msg.twist.linear.x], [msg.twist.angular.z]])
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]
        self.command_pub_.publish(wheel_speed_msg)

    def jointCallback(self, msg):
        dp_left = msg.position[1] - self.left_wheel_previous_pos_
        dp_right = msg.position[0] - self.right_wheel_previous_pos_
        dt = Time.from_msg(msg.header.stamp) - self.previous_time_

        self.left_wheel_previous_pos_ = msg.position[1]
        self.right_wheel_previous_pos_ = msg.position[0]
        self.previous_time_ = Time.from_msg(msg.header.stamp)

        fi_left = dp_left / dt.nanoseconds / S_TO_NS
        fi_right = dp_right / dt.nanoseconds / S_TO_NS

        linear = self.wheel_radius_ * (fi_right + fi_left) / 2
        angular = self.wheel_radius_ * (fi_right - fi_left) / self.wheel_separation_

        d_s = self.wheel_radius_ * (dp_right + dp_left) / 2
        d_theta = self.wheel_radius_ * (dp_right - dp_left) / self.wheel_separation_

        self.theta += d_theta
        self.x_ += d_s * math.cos(self.theta)
        self.y_ += d_s * math.sin(self.theta)

        self.get_logger().info(
            f"""
        \nlinear: {linear}
        \nangular: {angular}
        \nx: {self.x_}
        \ny: {self.y_}
        \ntheta: {self.theta}
        """
        )

        q = quaternion_from_euler(0.0,0.0,self.theta)
        self.odom.pose.pose.orientation.x = q[0]
        self.odom.pose.pose.orientation.y = q[1]
        self.odom.pose.pose.orientation.z = q[2]
        self.odom.pose.pose.orientation.w = q[3]
        self.odom.pose.pose.position.x = self.x_
        self.odom.pose.pose.position.y = self.y_
        self.odom.header.stamp = self.get_clock().now().to_msg()
        self.odom.twist.twist.linear.x = linear
        self.odom.twist.twist.angular.z = angular

        self.odom_pub_.publish(self.odom)

def main(args=None):
    rclpy.init(args=args)
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
