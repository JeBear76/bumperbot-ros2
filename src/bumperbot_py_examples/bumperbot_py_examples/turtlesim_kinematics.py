import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtleSimKinematics(Node):
    def __init__(self):
        super().__init__('turtlesim_kinematics')
        self.get_logger().info('Turtlesim Kinematics Node has been started.')
        self.turtle1_pose_sub_ = self.create_subscription(
            msg_type=Pose,
            topic='/turtle1/pose',
            callback=self.turtle1_pose_callback,
            qos_profile=10
        )
        self.turtle1_pose_sub_ = self.create_subscription(
            msg_type=Pose,
            topic='/turtle2/pose',
            callback=self.turtle2_pose_callback,
            qos_profile=10
        )
        self.last_turtle1_pose_ = Pose()
        self.last_turtle2_pose_ = Pose()

    def turtle1_pose_callback(self, msg):
        #self.get_logger().info(f'Turtle1 Pose: x={msg.x}, y={msg.y}, theta={msg.theta}')
        self.last_turtle1_pose_ = msg

    def turtle2_pose_callback(self, msg):
        #self.get_logger().info(f'Turtle2 Pose: x={msg.x}, y={msg.y}, theta={msg.theta}')
        self.last_turtle2_pose_ = msg
        Tx = self.last_turtle2_pose_.x - self.last_turtle1_pose_.x
        Ty = self.last_turtle2_pose_.y - self.last_turtle1_pose_.y

        Ttheta = self.last_turtle2_pose_.theta - self.last_turtle1_pose_.theta
        Ttheta_deg = Ttheta * 180 / math.pi
        self.get_logger().info(f'''
                               Turtle translation:\n\t
                               Tx: {Tx} \n\t
                               Ty: {Ty}\n\t
                               Ttheta: {Ttheta_deg} degrees\n
                               Rotation Matrix:\n\t
                               |cos(Ttheta) -sin(Ttheta)|:| {math.cos(Ttheta)}\t{-math.sin(Ttheta)}|\n\t
                               |sin(Ttheta)  cos(Ttheta)|:| {-math.sin(Ttheta)}\t{math.cos(Ttheta)}|\n
                               ''')

def main(args=None):
    rclpy.init(args=args)
    turtlesim_kinematics = TurtleSimKinematics()
    rclpy.spin(turtlesim_kinematics)
    turtlesim_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()