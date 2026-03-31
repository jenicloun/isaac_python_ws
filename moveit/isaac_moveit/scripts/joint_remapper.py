#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Isaac Sim에서 나오는 joint 이름 → MoveIt에서 사용하는 joint 이름 매핑
JOINT_NAME_MAP = {
    "panda_joint1": "franka_1_panda_joint1",
    "panda_joint2": "franka_1_panda_joint2",
    "panda_joint3": "franka_1_panda_joint3",
    "panda_joint4": "franka_1_panda_joint4",
    "panda_joint5": "franka_1_panda_joint5",
    "panda_joint6": "franka_1_panda_joint6",
    "panda_joint7": "franka_1_panda_joint7",
    "panda_finger_joint1": "franka_1_panda_finger_joint1",
    "panda_finger_joint2": "franka_1_panda_finger_joint2",
}

class JointStateRemapper(Node):
    def __init__(self):
        super().__init__('joint_state_remapper')
        self.subscription = self.create_subscription(
            JointState,
            '/franka_1_isaac_joint_states',  # Isaac Sim 토픽
            self.callback,
            10
        )
        self.publisher = self.create_publisher(
            JointState,
            '/franka_1/joint_states',  # MoveIt/RViz에서 읽을 토픽
            10
        )

    def callback(self, msg: JointState):
        new_msg = JointState()
        new_msg.header = msg.header
        new_msg.name = [JOINT_NAME_MAP.get(name, name) for name in msg.name]
        new_msg.position = msg.position
        new_msg.velocity = msg.velocity
        new_msg.effort = msg.effort
        self.publisher.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRemapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()