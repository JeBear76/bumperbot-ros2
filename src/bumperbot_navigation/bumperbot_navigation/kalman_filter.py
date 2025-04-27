#!/usr/bin/env python3
import rclpy
from rclpy import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class KalmanFilter(Node):
    def __init__(self):
        super().__init__("kalman_filter")
        self.odom_sub_ = self.create_subscription(Odometry, "/bumperbot_controller/noisy_odom", self.odomCallback, 10)
        self.imu_sub_ = self.create_subscription(Imu, "/imu/out", self.imuCallback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, "/bumperbot_controller/odom_kalman")
        
        self.mean_ = 0.0
        self.variance_ = 1000.0

        self.imu_angular_z_ = 0.0
        self.is_first_odom_ = True
        self.last_angular_z_ = 0.0

        self.motion_ = 0.0

        self.kalman_odom_ = Odometry()

    def odomCallback(self,msg):
        self.kalman_odom_ = msg
        if self.is_first_odom_:
            self.mean_ = msg.twist.twist.angular.z
            self.last_angular_z_ = msg.twist.twist.angular.z
            self.is_first_odom_ = False
            return
        self.statePrediction()
        self.measurementUpdate()

    def imuCallback(self,msg):
        self.imu_angular_z_ = msg.angular_velocity.z

    def statePrediction(self):
        pass
        
    def measurementUpdate(self):
        pass
