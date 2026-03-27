# launch/mapf_goal_checker.py
#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class SingleRobotGoalChecker(Node):
    def __init__(self):
        super().__init__("single_robot_goal_checker")

        self.declare_parameter("robot_name", "carter1")
        self.declare_parameter("plan_topic", "/mapf/carter1/plan")
        self.declare_parameter("goal_reached_topic", "/mapf/carter1/goal_reached")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "carter1/chassis_link/base_link")
        self.declare_parameter("xy_tolerance", 0.25)
        self.declare_parameter("yaw_tolerance", 0.35)
        self.declare_parameter("check_rate", 10.0)
        self.declare_parameter("latch_success", True)

        self.robot_name = self.get_parameter("robot_name").value
        self.plan_topic = self.get_parameter("plan_topic").value
        self.goal_reached_topic = self.get_parameter("goal_reached_topic").value
        self.global_frame = self.get_parameter("global_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.xy_tolerance = float(self.get_parameter("xy_tolerance").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self.check_rate = float(self.get_parameter("check_rate").value)
        self.latch_success = bool(self.get_parameter("latch_success").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.final_goal: Optional[PoseStamped] = None
        self.goal_reached_latched = False
        self.last_state: Optional[bool] = None

        self.plan_sub = self.create_subscription(
            Path,
            self.plan_topic,
            self.plan_callback,
            10,
        )

        self.goal_pub = self.create_publisher(
            Bool,
            self.goal_reached_topic,
            10,
        )

        self.timer = self.create_timer(1.0 / max(self.check_rate, 1.0), self.timer_callback)

        self.get_logger().info(
            f"[{self.robot_name}] checker started | "
            f"plan_topic={self.plan_topic}, goal_reached_topic={self.goal_reached_topic}, "
            f"frame={self.global_frame}->{self.base_frame}"
        )

    def plan_callback(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn(f"[{self.robot_name}] Received empty Path")
            self.final_goal = None
            self.goal_reached_latched = False
            return

        self.final_goal = msg.poses[-1]
        self.goal_reached_latched = False

        gx = self.final_goal.pose.position.x
        gy = self.final_goal.pose.position.y
        gyaw = yaw_from_quat(self.final_goal.pose.orientation)

        self.get_logger().info(
            f"[{self.robot_name}] New final goal received: "
            f"x={gx:.3f}, y={gy:.3f}, yaw={gyaw:.3f}"
        )

    def get_current_transform(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            return tf.transform
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().debug(
                f"[{self.robot_name}] TF lookup failed ({self.global_frame} <- {self.base_frame}): {e}"
            )
            return None

    def compute_goal_reached(self) -> bool:
        if self.final_goal is None:
            return False

        if self.latch_success and self.goal_reached_latched:
            return True

        tf_transform = self.get_current_transform()
        if tf_transform is None:
            return False

        cx = tf_transform.translation.x
        cy = tf_transform.translation.y
        cyaw = yaw_from_quat(tf_transform.rotation)

        gx = self.final_goal.pose.position.x
        gy = self.final_goal.pose.position.y
        gyaw = yaw_from_quat(self.final_goal.pose.orientation)

        dx = cx - gx
        dy = cy - gy
        dist = math.sqrt(dx * dx + dy * dy)
        yaw_err = abs(normalize_angle(cyaw - gyaw))

        reached = (dist <= self.xy_tolerance) and (yaw_err <= self.yaw_tolerance)

        if reached and self.latch_success:
            self.goal_reached_latched = True

        return reached

    def timer_callback(self):
        reached = self.compute_goal_reached()

        msg = Bool()
        msg.data = reached
        self.goal_pub.publish(msg)

        if self.last_state != reached:
            self.get_logger().info(
                f"[{self.robot_name}] goal_reached={reached}"
            )
            self.last_state = reached


def main(args=None):
    rclpy.init(args=args)
    node = SingleRobotGoalChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()