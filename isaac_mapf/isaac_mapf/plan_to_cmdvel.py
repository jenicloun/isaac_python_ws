# import math
# from typing import Optional

# import rclpy
# from rclpy.node import Node
# from rclpy.duration import Duration

# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Path
# from tf2_ros import Buffer, TransformListener, TransformException


# def clamp(value: float, low: float, high: float) -> float:
#     return max(low, min(high, value))


# def yaw_from_quaternion(q) -> float:
#     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#     return math.atan2(siny_cosp, cosy_cosp)


# def normalize_angle(angle: float) -> float:
#     while angle > math.pi:
#         angle -= 2.0 * math.pi
#     while angle < -math.pi:
#         angle += 2.0 * math.pi
#     return angle


# class PlanToCmdVel(Node):
#     def __init__(self):
#         super().__init__("plan_to_cmdvel")

#         self.declare_parameter("plan_topic", "/mapf/carter1/plan")
#         self.declare_parameter("cmd_vel_topic", "/carter1/cmd_vel")
#         self.declare_parameter("global_frame", "map")
#         self.declare_parameter("base_frame", "carter1/chassis_link/base_link")

#         self.declare_parameter("control_rate", 10.0)
#         self.declare_parameter("lookahead_index", 5)
#         self.declare_parameter("goal_tolerance", 0.2)

#         self.declare_parameter("linear_k", 0.8)
#         self.declare_parameter("angular_k", 0.8)
#         self.declare_parameter("max_linear", 0.5)
#         self.declare_parameter("max_angular", 0.6)

#         self.declare_parameter("rotate_enter_threshold", 0.8)
#         self.declare_parameter("rotate_exit_threshold", 0.35)
#         self.declare_parameter("slow_down_distance", 1.0)

#         self.declare_parameter("nearest_search_back", 3)
#         self.declare_parameter("nearest_search_forward", 20)
#         self.declare_parameter("heading_slowdown_threshold", 0.4)
#         self.declare_parameter("heading_slowdown_linear_cap", 0.15)

#         self.plan_topic = self.get_parameter("plan_topic").get_parameter_value().string_value
#         self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
#         self.global_frame = self.get_parameter("global_frame").get_parameter_value().string_value
#         self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value

#         self.control_rate = self.get_parameter("control_rate").get_parameter_value().double_value
#         self.lookahead_index = self.get_parameter("lookahead_index").get_parameter_value().integer_value
#         self.goal_tolerance = self.get_parameter("goal_tolerance").get_parameter_value().double_value

#         self.linear_k = self.get_parameter("linear_k").get_parameter_value().double_value
#         self.angular_k = self.get_parameter("angular_k").get_parameter_value().double_value
#         self.max_linear = self.get_parameter("max_linear").get_parameter_value().double_value
#         self.max_angular = self.get_parameter("max_angular").get_parameter_value().double_value

#         self.rotate_enter_threshold = (
#             self.get_parameter("rotate_enter_threshold").get_parameter_value().double_value
#         )
#         self.rotate_exit_threshold = (
#             self.get_parameter("rotate_exit_threshold").get_parameter_value().double_value
#         )
#         self.slow_down_distance = (
#             self.get_parameter("slow_down_distance").get_parameter_value().double_value
#         )

#         self.nearest_search_back = (
#             self.get_parameter("nearest_search_back").get_parameter_value().integer_value
#         )
#         self.nearest_search_forward = (
#             self.get_parameter("nearest_search_forward").get_parameter_value().integer_value
#         )
#         self.heading_slowdown_threshold = (
#             self.get_parameter("heading_slowdown_threshold").get_parameter_value().double_value
#         )
#         self.heading_slowdown_linear_cap = (
#             self.get_parameter("heading_slowdown_linear_cap").get_parameter_value().double_value
#         )

#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         self.path_sub = self.create_subscription(Path, self.plan_topic, self.path_callback, 10)
#         self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

#         self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

#         self.current_path: Optional[Path] = None
#         self.last_target_index: int = 0
#         self.goal_reached: bool = False
#         self.rotate_mode: bool = False
#         self.last_path_frame_warn: Optional[str] = None

#         self.get_logger().info(f"plan_topic              = {self.plan_topic}")
#         self.get_logger().info(f"cmd_vel_topic           = {self.cmd_vel_topic}")
#         self.get_logger().info(f"global_frame            = {self.global_frame}")
#         self.get_logger().info(f"base_frame              = {self.base_frame}")
#         self.get_logger().info(f"control_rate            = {self.control_rate}")
#         self.get_logger().info(f"lookahead_index         = {self.lookahead_index}")
#         self.get_logger().info(f"goal_tolerance          = {self.goal_tolerance}")
#         self.get_logger().info(f"linear_k                = {self.linear_k}")
#         self.get_logger().info(f"angular_k               = {self.angular_k}")
#         self.get_logger().info(f"max_linear              = {self.max_linear}")
#         self.get_logger().info(f"max_angular             = {self.max_angular}")
#         self.get_logger().info(f"rotate_enter_threshold  = {self.rotate_enter_threshold}")
#         self.get_logger().info(f"rotate_exit_threshold   = {self.rotate_exit_threshold}")
#         self.get_logger().info(f"slow_down_distance      = {self.slow_down_distance}")
#         self.get_logger().info(f"nearest_search_back     = {self.nearest_search_back}")
#         self.get_logger().info(f"nearest_search_forward  = {self.nearest_search_forward}")
#         self.get_logger().info(f"heading_slowdown_thresh = {self.heading_slowdown_threshold}")
#         self.get_logger().info(f"heading_slowdown_cap    = {self.heading_slowdown_linear_cap}")

#     def path_callback(self, msg: Path):
#         if len(msg.poses) == 0:
#             self.get_logger().warn("Received empty path. Ignoring.")
#             return

#         if self.same_path(self.current_path, msg):
#             return

#         frame_id = msg.header.frame_id
#         if frame_id and frame_id != self.global_frame:
#             self.get_logger().warn(
#                 f"Path frame mismatch: path={frame_id}, controller={self.global_frame}"
#             )

#         for i, ps in enumerate(msg.poses):
#             pf = ps.header.frame_id
#             if pf and pf != self.global_frame:
#                 self.get_logger().warn(
#                     f"Pose frame mismatch at idx={i}: pose={pf}, controller={self.global_frame}"
#                 )
#                 break

#         old_path = self.current_path
#         self.current_path = msg

#         if old_path is None:
#             self.last_target_index = 0
#         else:
#             self.last_target_index = min(self.last_target_index, len(msg.poses) - 1)

#         self.goal_reached = False
#         self.rotate_mode = False

#         self.get_logger().info(
#             f"Received NEW path with {len(msg.poses)} poses, frame={msg.header.frame_id}"
#         )

#     def get_current_pose(self):
#         try:
#             tf = self.tf_buffer.lookup_transform(
#                 self.global_frame,
#                 self.base_frame,
#                 rclpy.time.Time(),
#                 timeout=Duration(seconds=0.2),
#             )
#             x = tf.transform.translation.x
#             y = tf.transform.translation.y
#             q = tf.transform.rotation
#             yaw = yaw_from_quaternion(q)
#             return x, y, yaw
#         except TransformException as ex:
#             self.get_logger().warn(f"TF lookup failed: {ex}", throttle_duration_sec=2.0)
#             return None

#     def publish_stop(self):
#         msg = Twist()
#         self.cmd_pub.publish(msg)

#     def control_loop(self):
#         if self.current_path is None or len(self.current_path.poses) == 0:
#             self.publish_stop()
#             return

#         # if self.current_path.header.frame_id and self.current_path.header.frame_id != self.global_frame:
#         #     if self.last_path_frame_warn != self.current_path.header.frame_id:
#         #         self.get_logger().warn(
#         #             f"Current path frame is {self.current_path.header.frame_id}, "
#         #             f"but controller expects {self.global_frame}. Stopping."
#         #         )
#         #         self.last_path_frame_warn = self.current_path.header.frame_id
#         #     self.publish_stop()
#         #     return

#         if not self.validate_pose_frames(self.current_path):
#             self.publish_stop()
#             return
        
#         pose = self.get_current_pose()
#         if pose is None:
#             self.publish_stop()
#             return

#         robot_x, robot_y, robot_yaw = pose
#         path_poses = self.current_path.poses

#         goal_pose = path_poses[-1].pose
#         goal_dx = goal_pose.position.x - robot_x
#         goal_dy = goal_pose.position.y - robot_y
#         goal_dist = math.hypot(goal_dx, goal_dy)

#         if goal_dist < self.goal_tolerance:
#             if not self.goal_reached:
#                 self.get_logger().info("Goal reached. Publishing stop.")
#                 self.goal_reached = True
#             self.publish_stop()
#             return

#         search_start = max(0, self.last_target_index - self.nearest_search_back)
#         search_end = min(len(path_poses), self.last_target_index + self.nearest_search_forward)

#         if search_start >= search_end:
#             search_start = 0
#             search_end = len(path_poses)

#         nearest_idx = search_start
#         nearest_dist = float("inf")
#         for i in range(search_start, search_end):
#             ps = path_poses[i]
#             dx = ps.pose.position.x - robot_x
#             dy = ps.pose.position.y - robot_y
#             d = dx * dx + dy * dy
#             if d < nearest_dist:
#                 nearest_dist = d
#                 nearest_idx = i

#         target_idx = min(nearest_idx + self.lookahead_index, len(path_poses) - 1)
#         self.last_target_index = target_idx
#         target_pose = path_poses[target_idx].pose

#         target_dx = target_pose.position.x - robot_x
#         target_dy = target_pose.position.y - robot_y
#         target_dist = math.hypot(target_dx, target_dy)

#         target_yaw = math.atan2(target_dy, target_dx)
#         yaw_error = normalize_angle(target_yaw - robot_yaw)

#         if self.rotate_mode:
#             if abs(yaw_error) < self.rotate_exit_threshold:
#                 self.rotate_mode = False
#         else:
#             if abs(yaw_error) > self.rotate_enter_threshold:
#                 self.rotate_mode = True

#         cmd = Twist()

#         if self.rotate_mode:
#             cmd.linear.x = 0.0
#             cmd.angular.z = clamp(
#                 self.angular_k * yaw_error,
#                 -self.max_angular,
#                 self.max_angular,
#             )
#         else:
#             speed_scale = 1.0
#             if goal_dist < self.slow_down_distance:
#                 speed_scale = max(0.2, goal_dist / self.slow_down_distance)

#             heading_scale = max(0.0, math.cos(yaw_error)) ** 2

#             linear = self.linear_k * target_dist * heading_scale
#             angular = self.angular_k * yaw_error

#             cmd.linear.x = clamp(linear, 0.0, self.max_linear) * speed_scale
#             cmd.angular.z = clamp(angular, -self.max_angular, self.max_angular)

#             if abs(yaw_error) > self.heading_slowdown_threshold:
#                 cmd.linear.x = min(cmd.linear.x, self.heading_slowdown_linear_cap)

#         self.cmd_pub.publish(cmd)

#     def validate_pose_frames(self, path: Path) -> bool:
#         if path.header.frame_id and path.header.frame_id != self.global_frame:
#             self.get_logger().warn(
#                 f"Current path frame is {path.header.frame_id}, but controller expects {self.global_frame}. Stopping."
#             )
#             return False

#         for i, ps in enumerate(path.poses):
#             pf = ps.header.frame_id
#             if pf and pf != self.global_frame:
#                 self.get_logger().warn(
#                     f"Pose[{i}] frame is {pf}, but controller expects {self.global_frame}. Stopping."
#                 )
#                 return False

#         return True

#     def same_path(self, a: Optional[Path], b: Optional[Path], pos_eps: float = 1e-6) -> bool:
#         if a is None or b is None:
#             return False

#         if a.header.frame_id != b.header.frame_id:
#             return False

#         if len(a.poses) != len(b.poses):
#             return False

#         for pa, pb in zip(a.poses, b.poses):
#             if pa.header.frame_id != pb.header.frame_id:
#                 return False

#             ax = pa.pose.position.x
#             ay = pa.pose.position.y
#             az = pa.pose.position.z

#             bx = pb.pose.position.x
#             by = pb.pose.position.y
#             bz = pb.pose.position.z

#             if abs(ax - bx) > pos_eps or abs(ay - by) > pos_eps or abs(az - bz) > pos_eps:
#                 return False

#             qa = pa.pose.orientation
#             qb = pb.pose.orientation
#             if (
#                 abs(qa.x - qb.x) > pos_eps or
#                 abs(qa.y - qb.y) > pos_eps or
#                 abs(qa.z - qb.z) > pos_eps or
#                 abs(qa.w - qb.w) > pos_eps
#             ):
#                 return False

#         return True


#     def first_pose_too_far(self, robot_x: float, robot_y: float, path: Path, threshold: float = 3.0) -> bool:
#         if len(path.poses) == 0:
#             return True
#         p0 = path.poses[0].pose.position
#         d = math.hypot(p0.x - robot_x, p0.y - robot_y)
#         return d > threshold
    
# def main(args=None):
#     rclpy.init(args=args)
#     node = PlanToCmdVel()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.publish_stop()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()


import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener, TransformException


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PlanToCmdVel(Node):
    def __init__(self):
        super().__init__("plan_to_cmdvel")

        self.declare_parameter("plan_topic", "/mapf/carter1/plan")
        self.declare_parameter("cmd_vel_topic", "/carter1/cmd_vel")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "carter1/chassis_link/base_link")

        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("lookahead_index", 5)
        self.declare_parameter("goal_tolerance", 0.2)

        self.declare_parameter("linear_k", 0.8)
        self.declare_parameter("angular_k", 0.8)
        self.declare_parameter("max_linear", 0.5)
        self.declare_parameter("max_angular", 0.6)

        self.declare_parameter("rotate_enter_threshold", 0.8)
        self.declare_parameter("rotate_exit_threshold", 0.35)
        self.declare_parameter("slow_down_distance", 1.0)

        self.declare_parameter("nearest_search_back", 3)
        self.declare_parameter("nearest_search_forward", 20)
        self.declare_parameter("heading_slowdown_threshold", 0.4)
        self.declare_parameter("heading_slowdown_linear_cap", 0.15)

        self.declare_parameter("path_start_max_distance", 3.0)
        self.declare_parameter("debug_print_interval", 1.0)

        self.plan_topic = self.get_parameter("plan_topic").get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.global_frame = self.get_parameter("global_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value

        self.control_rate = self.get_parameter("control_rate").get_parameter_value().double_value
        self.lookahead_index = self.get_parameter("lookahead_index").get_parameter_value().integer_value
        self.goal_tolerance = self.get_parameter("goal_tolerance").get_parameter_value().double_value

        self.linear_k = self.get_parameter("linear_k").get_parameter_value().double_value
        self.angular_k = self.get_parameter("angular_k").get_parameter_value().double_value
        self.max_linear = self.get_parameter("max_linear").get_parameter_value().double_value
        self.max_angular = self.get_parameter("max_angular").get_parameter_value().double_value

        self.rotate_enter_threshold = (
            self.get_parameter("rotate_enter_threshold").get_parameter_value().double_value
        )
        self.rotate_exit_threshold = (
            self.get_parameter("rotate_exit_threshold").get_parameter_value().double_value
        )
        self.slow_down_distance = (
            self.get_parameter("slow_down_distance").get_parameter_value().double_value
        )

        self.nearest_search_back = (
            self.get_parameter("nearest_search_back").get_parameter_value().integer_value
        )
        self.nearest_search_forward = (
            self.get_parameter("nearest_search_forward").get_parameter_value().integer_value
        )
        self.heading_slowdown_threshold = (
            self.get_parameter("heading_slowdown_threshold").get_parameter_value().double_value
        )
        self.heading_slowdown_linear_cap = (
            self.get_parameter("heading_slowdown_linear_cap").get_parameter_value().double_value
        )

        self.path_start_max_distance = (
            self.get_parameter("path_start_max_distance").get_parameter_value().double_value
        )
        self.debug_print_interval = (
            self.get_parameter("debug_print_interval").get_parameter_value().double_value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_sub = self.create_subscription(Path, self.plan_topic, self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.current_path: Optional[Path] = None
        self.last_target_index: int = 0
        self.goal_reached: bool = False
        self.rotate_mode: bool = False
        self.last_path_frame_warn: Optional[str] = None
        self.last_debug_time = self.get_clock().now()

        self.get_logger().info(f"plan_topic              = {self.plan_topic}")
        self.get_logger().info(f"cmd_vel_topic           = {self.cmd_vel_topic}")
        self.get_logger().info(f"global_frame            = {self.global_frame}")
        self.get_logger().info(f"base_frame              = {self.base_frame}")
        self.get_logger().info(f"control_rate            = {self.control_rate}")
        self.get_logger().info(f"lookahead_index         = {self.lookahead_index}")
        self.get_logger().info(f"goal_tolerance          = {self.goal_tolerance}")
        self.get_logger().info(f"linear_k                = {self.linear_k}")
        self.get_logger().info(f"angular_k               = {self.angular_k}")
        self.get_logger().info(f"max_linear              = {self.max_linear}")
        self.get_logger().info(f"max_angular             = {self.max_angular}")
        self.get_logger().info(f"rotate_enter_threshold  = {self.rotate_enter_threshold}")
        self.get_logger().info(f"rotate_exit_threshold   = {self.rotate_exit_threshold}")
        self.get_logger().info(f"slow_down_distance      = {self.slow_down_distance}")
        self.get_logger().info(f"nearest_search_back     = {self.nearest_search_back}")
        self.get_logger().info(f"nearest_search_forward  = {self.nearest_search_forward}")
        self.get_logger().info(f"heading_slowdown_thresh = {self.heading_slowdown_threshold}")
        self.get_logger().info(f"heading_slowdown_cap    = {self.heading_slowdown_linear_cap}")
        self.get_logger().info(f"path_start_max_distance = {self.path_start_max_distance}")
        self.get_logger().info(f"debug_print_interval    = {self.debug_print_interval}")

    def same_path(self, a: Optional[Path], b: Optional[Path], pos_eps: float = 1e-6) -> bool:
        if a is None or b is None:
            return False

        if a.header.frame_id != b.header.frame_id:
            return False

        if len(a.poses) != len(b.poses):
            return False

        for pa, pb in zip(a.poses, b.poses):
            if pa.header.frame_id != pb.header.frame_id:
                return False

            ap = pa.pose.position
            bp = pb.pose.position
            if (
                abs(ap.x - bp.x) > pos_eps
                or abs(ap.y - bp.y) > pos_eps
                or abs(ap.z - bp.z) > pos_eps
            ):
                return False

            ao = pa.pose.orientation
            bo = pb.pose.orientation
            if (
                abs(ao.x - bo.x) > pos_eps
                or abs(ao.y - bo.y) > pos_eps
                or abs(ao.z - bo.z) > pos_eps
                or abs(ao.w - bo.w) > pos_eps
            ):
                return False

        return True

    def validate_pose_frames(self, path: Path) -> bool:
        if path.header.frame_id and path.header.frame_id != self.global_frame:
            self.get_logger().warn(
                f"Current path frame is {path.header.frame_id}, "
                f"but controller expects {self.global_frame}. Stopping."
            )
            return False

        for i, ps in enumerate(path.poses):
            pf = ps.header.frame_id
            if pf and pf != self.global_frame:
                self.get_logger().warn(
                    f"Pose[{i}] frame is {pf}, but controller expects {self.global_frame}. Stopping."
                )
                return False

        return True

    def first_pose_too_far(
        self,
        robot_x: float,
        robot_y: float,
        path: Path,
        threshold: float,
    ) -> bool:
        if len(path.poses) == 0:
            return True
        p0 = path.poses[0].pose.position
        d = math.hypot(p0.x - robot_x, p0.y - robot_y)
        return d > threshold

    def path_callback(self, msg: Path):
        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty path. Ignoring.")
            return

        if self.same_path(self.current_path, msg):
            return

        frame_id = msg.header.frame_id
        if frame_id and frame_id != self.global_frame:
            self.get_logger().warn(
                f"Path frame mismatch: path={frame_id}, controller={self.global_frame}"
            )

        for i, ps in enumerate(msg.poses):
            pf = ps.header.frame_id
            if pf and pf != self.global_frame:
                self.get_logger().warn(
                    f"Pose frame mismatch at idx={i}: pose={pf}, controller={self.global_frame}"
                )
                break

        old_path = self.current_path
        self.current_path = msg

        if old_path is None:
            self.last_target_index = 0
        else:
            self.last_target_index = min(self.last_target_index, len(msg.poses) - 1)

        self.goal_reached = False
        self.rotate_mode = False

        first = msg.poses[0].pose.position
        last = msg.poses[-1].pose.position
        self.get_logger().info(
            f"Received NEW path with {len(msg.poses)} poses, "
            f"frame={msg.header.frame_id}, "
            f"first=({first.x:.3f}, {first.y:.3f}), "
            f"last=({last.x:.3f}, {last.y:.3f})"
        )

    def get_current_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = yaw_from_quaternion(q)
            return x, y, yaw
        except TransformException as ex:
            self.get_logger().warn(
                f"TF lookup failed: {ex}",
                throttle_duration_sec=2.0,
            )
            return None

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def maybe_debug_print(
        self,
        robot_x: float,
        robot_y: float,
        target_idx: int,
        target_x: float,
        target_y: float,
        goal_dist: float,
        yaw_error: float,
        cmd: Twist,
    ):
        now = self.get_clock().now()
        dt = (now - self.last_debug_time).nanoseconds / 1e9
        if dt < self.debug_print_interval:
            return

        self.last_debug_time = now
        self.get_logger().info(
            f"[track] robot=({robot_x:.3f},{robot_y:.3f}) "
            f"target_idx={target_idx} "
            f"target=({target_x:.3f},{target_y:.3f}) "
            f"goal_dist={goal_dist:.3f} "
            f"yaw_error={yaw_error:.3f} "
            f"cmd=({cmd.linear.x:.3f},{cmd.angular.z:.3f})"
        )

    def control_loop(self):
        if self.current_path is None or len(self.current_path.poses) == 0:
            self.publish_stop()
            return

        if not self.validate_pose_frames(self.current_path):
            self.publish_stop()
            return

        pose = self.get_current_pose()
        if pose is None:
            self.publish_stop()
            return

        robot_x, robot_y, robot_yaw = pose
        path_poses = self.current_path.poses

        if self.first_pose_too_far(
            robot_x,
            robot_y,
            self.current_path,
            threshold=self.path_start_max_distance,
        ):
            self.get_logger().warn(
                "Path start is too far from current robot pose. Stopping."
            )
            self.publish_stop()
            return

        goal_pose = path_poses[-1].pose
        goal_dx = goal_pose.position.x - robot_x
        goal_dy = goal_pose.position.y - robot_y
        goal_dist = math.hypot(goal_dx, goal_dy)

        if goal_dist < self.goal_tolerance:
            if not self.goal_reached:
                self.get_logger().info("Goal reached. Publishing stop.")
                self.goal_reached = True
            self.publish_stop()
            return

        search_start = max(0, self.last_target_index - self.nearest_search_back)
        search_end = min(len(path_poses), self.last_target_index + self.nearest_search_forward)

        if search_start >= search_end:
            search_start = 0
            search_end = len(path_poses)

        nearest_idx = search_start
        nearest_dist = float("inf")

        for i in range(search_start, search_end):
            ps = path_poses[i]
            dx = ps.pose.position.x - robot_x
            dy = ps.pose.position.y - robot_y
            d = dx * dx + dy * dy
            if d < nearest_dist:
                nearest_dist = d
                nearest_idx = i

        target_idx = min(nearest_idx + self.lookahead_index, len(path_poses) - 1)
        self.last_target_index = target_idx
        target_pose = path_poses[target_idx].pose

        target_dx = target_pose.position.x - robot_x
        target_dy = target_pose.position.y - robot_y
        target_dist = math.hypot(target_dx, target_dy)

        target_yaw = math.atan2(target_dy, target_dx)
        yaw_error = normalize_angle(target_yaw - robot_yaw)

        if self.rotate_mode:
            if abs(yaw_error) < self.rotate_exit_threshold:
                self.rotate_mode = False
        else:
            if abs(yaw_error) > self.rotate_enter_threshold:
                self.rotate_mode = True

        cmd = Twist()

        if self.rotate_mode:
            cmd.linear.x = 0.0
            cmd.angular.z = clamp(
                self.angular_k * yaw_error,
                -self.max_angular,
                self.max_angular,
            )
        else:
            speed_scale = 1.0
            if goal_dist < self.slow_down_distance:
                speed_scale = max(0.2, goal_dist / self.slow_down_distance)

            heading_scale = max(0.0, math.cos(yaw_error)) ** 2

            linear = self.linear_k * target_dist * heading_scale
            angular = self.angular_k * yaw_error

            cmd.linear.x = clamp(linear, 0.0, self.max_linear) * speed_scale
            cmd.angular.z = clamp(angular, -self.max_angular, self.max_angular)

            if abs(yaw_error) > self.heading_slowdown_threshold:
                cmd.linear.x = min(cmd.linear.x, self.heading_slowdown_linear_cap)

        self.cmd_pub.publish(cmd)

        self.maybe_debug_print(
            robot_x=robot_x,
            robot_y=robot_y,
            target_idx=target_idx,
            target_x=target_pose.position.x,
            target_y=target_pose.position.y,
            goal_dist=goal_dist,
            yaw_error=yaw_error,
            cmd=cmd,
        )


def main(args=None):
    rclpy.init(args=args)
    node = PlanToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()