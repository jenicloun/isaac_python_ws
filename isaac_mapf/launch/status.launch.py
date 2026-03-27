from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_mapf',
            executable='status_node',
            name='status_node',
            output='screen',
            parameters=[
                {"robot_name": "carter1"},
                {"period": 1.5}
            ]
        )
    ])