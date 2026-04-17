import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
import yaml
import os
import math
import time

class MultiGoalSender(Node):
    def __init__(self, yaml_file):
        super().__init__('multi_goal_sender')
        
        with open(yaml_file, 'r') as f:
            self.goals = yaml.safe_load(f)

        for robot_name, coords in self.goals.items():
            action_topic = f'/{robot_name}/navigate_to_pose'
            client = ActionClient(self, NavigateToPose, action_topic)
            
            self.get_logger().info(f'Waiting for {action_topic} server...')
            if client.wait_for_server(timeout_sec=5.0):
                self.send_goal(client, robot_name, coords)
            else:
                self.get_logger().error(f'Could not find {action_topic}!')

    def send_goal(self, client, name, coords):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(coords['x'])
        goal_msg.pose.pose.position.y = float(coords['y'])
        goal_msg.pose.pose.orientation.w = 1.0 # 간단하게 정면 응시
        # --- 추가 및 수정 부분 ---
        # 1. Degree를 Radian으로 변환
        yaw_deg = float(coords.get('yaw', 0.0))
        yaw_rad = math.radians(yaw_deg)
        
        # 2. Yaw(Z축 회전)를 Quaternion으로 변환하는 공식
        # Roll, Pitch가 0일 때: x=0, y=0, z=sin(yaw/2), w=cos(yaw/2)
        goal_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        # -----------------------
        
        self.get_logger().info(f'Sending goal to {name}: x={coords["x"]}, y={coords["y"]}')
        client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    current_file_path = os.path.abspath(__file__)
    script_dir = os.path.dirname(current_file_path)

    yaml_path = os.path.normpath(os.path.join(script_dir, '..', 'config', 'multi_goals.yaml'))
    
    node = MultiGoalSender(yaml_path)
 
    start_time = time.time()
    while rclpy.ok() and (time.time() - start_time) < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()