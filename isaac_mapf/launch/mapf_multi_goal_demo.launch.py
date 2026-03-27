from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_mapf',
            executable='goal_pose_publisher',
            name='carter1_goal_publisher',
            output='screen',
            parameters=[
                {'topic_name': '/mapf/carter1/goal'},
                {'frame_id': 'World'},
                {'x': 5.0},
                {'y': 5.0},
                {'yaw': 0.0},
                {'period': 1.0},
            ]
        ),
        Node(
            package='isaac_mapf',
            executable='goal_pose_publisher',
            name='carter2_goal_publisher',
            output='screen',
            parameters=[
                {'topic_name': '/mapf/carter2/goal'},
                {'frame_id': 'World'},
                {'x': 0.0},
                {'y': 0.0},
                {'yaw': 0.0},
                {'period': 1.0},
            ]
        ),
        Node(
            package='isaac_mapf',
            executable='goal_pose_publisher',
            name='carter3_goal_publisher',
            output='screen',
            parameters=[
                {'topic_name': '/mapf/carter3/goal'},
                {'frame_id': 'World'},
                {'x': 3.0},
                {'y': 3.0},
                {'yaw': 0.0},
                {'period': 1.0},
            ]
        ),
    ])