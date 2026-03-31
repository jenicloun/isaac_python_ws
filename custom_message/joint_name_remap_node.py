#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointRemap(Node):
    def __init__(self):
        super().__init__('joint_remap')
        self.sub = self.create_subscription(
            JointState,
            '/franka_1_isaac_joint_states',
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            JointState,
            '/franka_1/joint_states',
            10
        )
        self.get_logger().info("JointRemap node started")

    def callback(self, msg):
        new_msg = JointState()
        new_msg.header = msg.header
        new_msg.name = ["franka_1_" + n for n in msg.name]
        new_msg.position = msg.position
        new_msg.velocity = msg.velocity
        new_msg.effort = msg.effort
        self.pub.publish(new_msg)

def main():
    rclpy.init()
    node = JointRemap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()