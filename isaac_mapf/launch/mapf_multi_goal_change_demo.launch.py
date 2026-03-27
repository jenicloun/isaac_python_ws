import math
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, LogInfo


def pub_goal(topic: str, x: float, y: float, yaw: float = 0.0):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)

    msg = (
        "{header: {frame_id: 'map'}, "
        "pose: {position: {x: " + str(x) + ", y: " + str(y) + ", z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: " + str(qz) + ", w: " + str(qw) + "}}}"
    )

    return ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            topic,
            'geometry_msgs/msg/PoseStamped',
            msg
        ],
        output='screen'
    )


def pub_flag():
    return ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/mapf/goal_init_flag',
            'std_msgs/msg/Bool',
            '{data: true}'
        ],
        output='screen'
    )


def generate_launch_description():
    return LaunchDescription([
        # 초기 goal publish
        TimerAction(
            period=1.0,
            actions=[
                LogInfo(msg='[mapf_goal_demo] Publishing initial goals'),
                pub_goal('/mapf/carter1/goal', 5.0, 5.0, 0.0),
                pub_goal('/mapf/carter2/goal', -2.475, -4.475, 0.0),
                pub_goal('/mapf/carter3/goal', -0.975, -2.975, 0.0),
            ]
        ),

        # 초기 goal들을 넣은 뒤 flag publish
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='[mapf_goal_demo] Publishing initial goal_init_flag'),
                pub_flag(),
            ]
        ),

        # 새 goal로 변경
        TimerAction(
            period=60.0,
            actions=[
                LogInfo(msg='[mapf_goal_demo] Changing goals for all agents'),
                pub_goal('/mapf/carter1/goal', 5.0, 0.0, 0.0),
                pub_goal('/mapf/carter2/goal', 3.0, 4.0, 0.0),
                pub_goal('/mapf/carter3/goal', -5.0, 0.0, 0.0),
            ]
        ),

        # 새 goal들을 넣은 뒤 다시 flag publish
        TimerAction(
            period=22.0,
            actions=[
                LogInfo(msg='[mapf_goal_demo] Publishing updated goal_init_flag'),
                pub_flag(),
            ]
        ),
        
        
    ])