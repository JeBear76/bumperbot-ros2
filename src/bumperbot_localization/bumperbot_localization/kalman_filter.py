#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class KalmanFilter(Node):
    def __init__(self):
        super().__init__("kalman_filter")
        self.odom_sub_ = self.create_subscription(Odometry, "/bumperbot_controller/odom_noisy", self.odomCallback, 10)
        self.imu_sub_ = self.create_subscription(Imu, "/imu/out", self.imuCallback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, "/bumperbot_controller/odom_kalman")
        
        self.mean_ = 0.0
        self.variance_ = 1000.0

        self.imu_angular_z_ = 0.0
        self.is_first_odom_ = True
        self.last_angular_z_ = 0.0

        self.motion_ = 0.0

        self.kalman_odom_ = Odometry()

        self.motion_variance_ = 4.0
        self.measurement_variance_ = 0.5

    def odomCallback(self,msg):
        self.kalman_odom_ = msg
        if self.is_first_odom_:
            self.mean_ = msg.twist.twist.angular.z
            self.last_angular_z_ = msg.twist.twist.angular.z
            self.is_first_odom_ = False
            return
        
        self.motion_ = msg.twist.twist.z - self.last_angular_z_

        self.statePrediction()
        self.measurementUpdate()

        self.kalman_odom_.twist.twist.angular.z = self.mean_

        self.odom_pub_.publish(self.kalman_odom_)

        self.last_angular_z_ = msg.twist.twist.z


    def imuCallback(self,msg):
        self.imu_angular_z_ = msg.angular_velocity.z
        
    def measurementUpdate(self):
        self.mean_ = (self.measurement_variance_ * self.mean_ + self.variance_ * self.imu_angular_z_)\
                     / (self.variance_ + self.measurement_variance_)
        self.variance_ = (self.variance_ * self.measurement_variance_)\
                     / (self.variance_ + self.measurement_variance_)

    def statePrediction(self):
        self.mean_ = self.mean_ + self.motion_
        self.variance_ = self.variance_ + self.motion_variance_

def main(args=None):
    rclpy.init(args=args)
    kalman_filter = KalmanFilter()
    rclpy.spin(kalman_filter)
    kalman_filter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()