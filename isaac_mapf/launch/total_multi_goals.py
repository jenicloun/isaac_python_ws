import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
import yaml
import os
from ament_index_python.packages import get_package_share_directory


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
        
        self.get_logger().info(f'Sending goal to {name}: x={coords["x"]}, y={coords["y"]}')
        client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    current_file_path = os.path.abspath(__file__)
    script_dir = os.path.dirname(current_file_path)
    
    # 2. launch 폴더의 상위로 이동(..)해서 config/multi_goals.yaml 경로를 만듭니다.
    # 구조: isaac_mapf/launch/.. -> isaac_mapf/config/multi_goals.yaml
    yaml_path = os.path.normpath(os.path.join(script_dir, '..', 'config', 'multi_goals.yaml'))
    
    node = MultiGoalSender(yaml_path)
    # 비동기로 목표만 쏘고 종료하려면 바로 shutdown, 
    # 결과를 기다리려면 spin을 사용하지만 여기서는 전송만 하고 종료합니다.
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()