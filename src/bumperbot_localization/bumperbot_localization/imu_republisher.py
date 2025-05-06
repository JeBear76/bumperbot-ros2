#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time;

class ImuRepublisher(Node):
    def __init__(self):
        super().__init__("imu_republisher")
        time.sleep(1)
        self.imu_sub_ = self.create_subscription(Imu, "/imu/out", self.imuCallback, 10)
        self.imu_pub_ = self.create_publisher(Imu, "/imu_ekf", 10)

    def imuCallback(self, msg):
        msg.header.frame_id = "base_footprint_ekf"
        self.imu_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    imu_republisher = ImuRepublisher()
    rclpy.spin(imu_republisher)
    imu_republisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()