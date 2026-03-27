#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')

        self.declare_parameter('topic_name', '/goal_pose')
        self.declare_parameter('frame_id', 'World')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('period', 1.0)

        self.declare_parameter('publish_goal_init_flag', False)
        self.declare_parameter('goal_init_flag_topic', '/mapf/goal_init_flag')

        self.declare_parameter('change_after', -1.0)
        self.declare_parameter('x2', 0.0)
        self.declare_parameter('y2', 0.0)
        self.declare_parameter('yaw2', 0.0)

        self.topic_name = self.get_parameter('topic_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.period = float(self.get_parameter('period').value)

        self.publish_goal_init_flag = bool(self.get_parameter('publish_goal_init_flag').value)
        self.goal_init_flag_topic = self.get_parameter('goal_init_flag_topic').value

        self.change_after = float(self.get_parameter('change_after').value)
        self.x2 = float(self.get_parameter('x2').value)
        self.y2 = float(self.get_parameter('y2').value)
        self.yaw2 = float(self.get_parameter('yaw2').value)

        self.publisher_ = self.create_publisher(PoseStamped, self.topic_name, 10)
        self.flag_pub_ = self.create_publisher(Bool, self.goal_init_flag_topic, 10)

        self.timer = self.create_timer(self.period, self.timer_callback)
        self.start_time = self.get_clock().now()
        self.changed = False
        self.last_phase = None

        self.get_logger().info(f'topic_name             = {self.topic_name}')
        self.get_logger().info(f'frame_id               = {self.frame_id}')
        self.get_logger().info(f'goal_1                 = ({self.x}, {self.y}, yaw={self.yaw})')
        self.get_logger().info(f'period                 = {self.period}')
        self.get_logger().info(f'publish_goal_init_flag = {self.publish_goal_init_flag}')
        self.get_logger().info(f'goal_init_flag_topic   = {self.goal_init_flag_topic}')
        self.get_logger().info(f'change_after           = {self.change_after}')
        self.get_logger().info(f'goal_2                 = ({self.x2}, {self.y2}, yaw={self.yaw2})')

    def yaw_to_quaternion(self, yaw: float):
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        return qz, qw

    def publish_flag(self):
        if not self.publish_goal_init_flag:
            return

        msg = Bool()
        msg.data = True
        self.flag_pub_.publish(msg)
        self.get_logger().info(f'Published goal_init_flag on {self.goal_init_flag_topic}')

    def publish_goal(self, x: float, y: float, yaw: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        qz, qw = self.yaw_to_quaternion(yaw)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.publisher_.publish(msg)

    def timer_callback(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        if self.change_after >= 0.0 and elapsed >= self.change_after:
            phase = "changed"
            x, y, yaw = self.x2, self.y2, self.yaw2
        else:
            phase = "initial"
            x, y, yaw = self.x, self.y, self.yaw

        self.publish_goal(x, y, yaw)

        if self.last_phase != phase:
            if phase == "initial":
                self.get_logger().info(f'Publishing initial goal: ({x}, {y}, yaw={yaw})')
            else:
                self.get_logger().info(f'Goal changed: ({x}, {y}, yaw={yaw})')
            self.publish_flag()
            self.last_phase = phase


def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()