#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist

class TwistRelay(Node):
    def __init__(self):
        super().__init__("twist_relay")
        self.declare_parameter("twist_in_topic", "bumperbot_controller/cmd_vel_unstamped")
        self.declare_parameter("twist_stamped_out_topic", "bumperbot_controller/cmd_vel")
        self.declare_parameter("twist_stamped_joy_in_topic", "input_joy/cmd_vel_stamped")
        self.declare_parameter("twist_joy_out_topic", "input_joy/cmd_vel")
        
        self.twist_sub = self.create_subscription(
            Twist,
            self.get_parameter("twist_in_topic").get_parameter_value().string_value,
            self.twist_callback,
            10,
        )

        self.twist_pub = self.create_publisher(
            TwistStamped, 
            self.get_parameter("twist_stamped_out_topic").get_parameter_value().string_value, 
            10
        )

        self.joy_sub = self.create_subscription(
            TwistStamped,
            self.get_parameter("twist_stamped_joy_in_topic").get_parameter_value().string_value,
            self.joy_callback,
            10,
        )

        self.joy_pub = self.create_publisher(
            Twist,
            self.get_parameter("twist_joy_out_topic").get_parameter_value().string_value,
            10,
        )

        
    def twist_callback(self, msg):
        # Create a new TwistStamped message
        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.twist = msg

        # Publish the TwistStamped message
        self.twist_pub.publish(twist_stamped_msg)

    def joy_callback(self, msg):
        # Create a new Twist message
        twist_msg = Twist()
        twist_msg.linear = msg.twist.linear
        twist_msg.angular = msg.twist.angular

        # Publish the Twist message
        self.joy_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    twist_relay = TwistRelay()
    rclpy.spin(twist_relay)
    twist_relay.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()