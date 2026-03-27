# /isaac_mapf/tf_prefix_relay.py
# mapf_base pkg는 /tf에서 각 로봇의 tf를 받기를 원하고 있음
# 근데 현재 isaac sim에서 publish하는 robot의 tf는 /carter1/tf와 같은 형식임
# 더 자세히, 각 agent가 모두 World -> nova_carter와 같은 형식으로 tf를 정의하였기 때문에
# 여러 에이전트의 tf를 구분할 수 없음
# 따라서 /carter1/tf -> /tf로 보내되 frame이름을 carter1/nova_carter와 같이 변경해야함

import copy

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TFPrefixRelay(Node):
    def __init__(self):
        super().__init__('tf_prefix_relay')
        
        self.declare_parameter('input_tf_topic', '/carter1/tf')
        self.declare_parameter('output_tf_topic', '/tf')
        self.declare_parameter('prefix', 'carter1')
        self.declare_parameter('global_frames', ['World'])

        self.input_tf_topic = self.get_parameter('input_tf_topic').value
        self.output_tf_topic = self.get_parameter('output_tf_topic').value
        self.prefix = self.get_parameter('prefix').value
        self.global_frames = set(self.get_parameter('global_frames').value)
        
        self.sub = self.create_subscription(
            TFMessage,
            self.input_tf_topic,
            self.tf_callback,
            50
        )
        
        self.pub = self.create_publisher(
            TFMessage,
            self.output_tf_topic,
            50
        )
                
        self.get_logger().info(f'input_tf_topic = {self.input_tf_topic}')
        self.get_logger().info(f'output_tf_topic = {self.output_tf_topic}')
        self.get_logger().info(f'prefix = {self.prefix}')
        self.get_logger().info(f'global_frames = {list(self.global_frames)}')
        
        
    # def add_prefix(self, frame_name: str) -> str:
    #     if not frame_name:
    #         return frame_name
        
    #     clean = frame_name.lstrip('/')
        
    #     if clean in self.global_frames:
    #         return clean
        
    #     if clean.startswith(f'{self.prefix}/'):
    #         return clean
        
    #     return f'{self.prefix}/{clean}'
    
    def add_prefix(self, frame_name: str) -> str:
        if not frame_name:
            return frame_name

        clean = frame_name.strip().lstrip('/')
        lowered = clean.lower()

        global_candidates = {g.strip().lstrip('/').lower() for g in self.global_frames}
        global_candidates.update({"world", "map", "odom"})

        if lowered in global_candidates:
            return clean

        if clean.startswith(f'{self.prefix}/'):
            return clean

        return f'{self.prefix}/{clean}'
    
    def tf_callback(self, msg: TFMessage):
        out = TFMessage()
        
        for i, t in enumerate(msg.transforms):
            new_t = copy.deepcopy(t)
            
            new_t.header.frame_id = self.add_prefix(new_t.header.frame_id)
            new_t.child_frame_id  = self.add_prefix(new_t.child_frame_id)
            
            out.transforms.append(new_t)

            # if i < 3:
            #     self.get_logger().info(
            #         f"relay tf: {new_t.header.frame_id} -> {new_t.child_frame_id}"
            #     )
        self.pub.publish(out)
    

def main(args=None):
    rclpy.init(args=args)
    node = TFPrefixRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()