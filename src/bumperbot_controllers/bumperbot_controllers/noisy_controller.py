#!/usr/bin/env python3
import rclpy
import numpy as np

import math

from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class NoisyController(Node):
    def __init__(self):
        super().__init__("noisy_controller")
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta = 0.0

        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_footprint_ekf"
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

        self.joint_subscriber_ = self.create_subscription(
            JointState, "joint_states", self.jointCallback, 10
        )

        self.odom_pub_ = self.create_publisher(
            Odometry, "bumperbot_controller/odom_noisy", 10
        )

        self.transform_broadcaster_ = TransformBroadcaster(self)

        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint_noisy"

        self.get_logger().info("Noisy Controller Node has been started.")

    def jointCallback(self, msg):
        # Messing with the values
        wheel_encoder_left = msg.position[1] + np.random.normal(0, 0.005)
        wheel_encoder_right = msg.position[0]+ np.random.normal(0, 0.005)

        dp_left = wheel_encoder_left - self.left_wheel_previous_pos_
        dp_right = wheel_encoder_right- self.right_wheel_previous_pos_
        dt = Time.from_msg(msg.header.stamp) - self.previous_time_

        # course material usues msg values instead of noisy values.
        # why?        
        self.left_wheel_previous_pos_ = wheel_encoder_left
        self.right_wheel_previous_pos_ = wheel_encoder_right

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

        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[2]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()

        self.transform_broadcaster_.sendTransform(self.transform_stamped_)

def main(args=None):
    rclpy.init(args=args)
    noisy_controller = NoisyController()
    rclpy.spin(noisy_controller)
    noisy_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
