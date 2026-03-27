import rclpy
from rclpy.node import Node

class StatusNode(Node):
    def __init__(self):
        super().__init__('status_node')
        
        self.declare_parameter('robot_name', 'Basket1')
        self.declare_parameter('period', 2.0)
        
        self.robot_name = self.get_parameter('robot_name').value
        self.period = float(self.get_parameter('period').value)
        
        self.timer = self.create_timer(self.period, self.timer_callback)
        
        self.get_logger().info(f'status_node start for robot: {self.robot_name}')
        self.get_logger().info(f'timer period: {self.period} sec')
        
    def timer_callback(self):
        self.get_logger().info(f'[{self.robot_name}] node is alive')
        

def main(args=None):
    rclpy.init(args=args)
    node = StatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
  
    
if __name__ == "__main__":
    main()
        