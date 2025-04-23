import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster

from geometry_msgs.msg import TransformStamped

class SimpleTFKinematics(Node):
    def __init__(self):
        super().__init__("simple_tf_kinematics")
        self.x_increment_ = 0.01
        self.last_x_ = 0.0

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)

        self.static_transform_stamped = TransformStamped()
        self.static_transform_stamped_1 = TransformStamped()
        self.dynamic_transform_stamped = TransformStamped()

        static_stamp_= self.get_clock().now().to_msg()
        self.static_transform_stamped.header.stamp = static_stamp_
        self.static_transform_stamped.header.frame_id = "bumperbot_base"
        self.static_transform_stamped.child_frame_id = "bumperbot_top"
        self.static_transform_stamped.transform.translation.x = 0.0
        self.static_transform_stamped.transform.translation.y = 0.0
        self.static_transform_stamped.transform.translation.z = 0.3
        self.static_transform_stamped.transform.rotation.x = 0.0
        self.static_transform_stamped.transform.rotation.y = 0.0
        self.static_transform_stamped.transform.rotation.z = 0.0
        self.static_transform_stamped.transform.rotation.w = 1.0

        self.static_transform_stamped_1.header.stamp = static_stamp_
        self.static_transform_stamped_1.header.frame_id = "bumperbot_top"
        self.static_transform_stamped_1.child_frame_id = "bumperbot_adjacent"
        self.static_transform_stamped_1.transform.translation.x = 0.3
        self.static_transform_stamped_1.transform.translation.y = 0.0
        self.static_transform_stamped_1.transform.translation.z = 0.0
        self.static_transform_stamped_1.transform.rotation.x = 0.0
        self.static_transform_stamped_1.transform.rotation.y = 0.0
        self.static_transform_stamped_1.transform.rotation.z = 0.0
        self.static_transform_stamped_1.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform([self.static_transform_stamped, self.static_transform_stamped_1])

        self.get_logger().info(f"Static transform between {self.static_transform_stamped.header.frame_id} and {self.static_transform_stamped.child_frame_id} published.")


        self.timer_ = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.dynamic_transform_stamped.header.stamp = self.get_clock().now().to_msg()        
        self.dynamic_transform_stamped.header.frame_id = "odom"
        self.dynamic_transform_stamped.child_frame_id = "bumperbot_base"
        self.dynamic_transform_stamped.transform.translation.x = self.last_x_ + self.x_increment_
        self.dynamic_transform_stamped.transform.translation.y = 0.0
        self.dynamic_transform_stamped.transform.translation.z = 0.0
        self.dynamic_transform_stamped.transform.rotation.x = 0.0
        self.dynamic_transform_stamped.transform.rotation.y = 0.0
        self.dynamic_transform_stamped.transform.rotation.z = 0.0
        self.dynamic_transform_stamped.transform.rotation.w = 1.0

        self.dynamic_tf_broadcaster.sendTransform(self.dynamic_transform_stamped)

        self.last_x_ = self.dynamic_transform_stamped.transform.translation.x

def main():
    rclpy.init()
    node = SimpleTFKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
