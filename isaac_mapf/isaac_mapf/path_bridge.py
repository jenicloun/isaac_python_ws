import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient

class PathBridge(Node):
    def __init__(self):
        super().__init__('path_bridge_node')
        self.namespace = 'carter1' # 테스트할 로봇 네임스페이스
        
        # MAPF가 발행하는 토픽 이름에 맞게 수정하세요 (예: /carter1/received_global_plan)
        self.path_topic = f'/mapf/{self.namespace}/plan' 
        
        # Nav2 Controller Server의 FollowPath 액션 클라이언트 생성
        self.action_client = ActionClient(self, FollowPath, f'/mapf/{self.namespace}/follow_path')
        
        # MAPF 경로 구독
        self.subscription = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10)
            
        self.get_logger().info(f"[{self.namespace}] Waiting for MAPF paths on {self.path_topic}...")

    def path_callback(self, msg):
        self.get_logger().info(f"Received path from MAPF! Sending to Nav2 Action Server...")
        
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('FollowPath action server not available!')
            return
            
        goal_msg = FollowPath.Goal()
        goal_msg.path = msg
        goal_msg.controller_id = 'FollowPath' # carter 파라미터에 설정된 컨트롤러 ID
        
        # Nav2로 액션 전송
        self.action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()