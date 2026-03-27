import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 설정: 네임스페이스 변수
    ns = "franka_1"

    # Command-line arguments
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="isaac",
        description="ROS2 control hardware interface type",
    )

    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true",
    )



    ns = "franka_1"
    # 본인 패키지의 설치 경로를 명확히 가져옵니다.

    my_pkg_share = get_package_share_directory("isaac_moveit")

    moveit_config = (
        # 첫 번째 인자는 로봇 이름, 두 번째는 패키지 이름입니다.
        MoveItConfigsBuilder("panda", package_name="isaac_moveit")
        .robot_description(
            # 절대 경로로 지정: os.path.join(패키지경로, "config", "파일명")
            file_path=os.path.join(my_pkg_share, "config", "panda.urdf.xacro"),
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration("ros2_control_hardware_type"),
            },
        )
        .robot_description_semantic(file_path=os.path.join(my_pkg_share, "config", "panda.srdf"))
        .trajectory_execution(file_path=os.path.join(my_pkg_share, "config", "gripper_moveit_controllers.yaml"))
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .joint_limits(file_path=os.path.join(my_pkg_share, "config", "joint_limits.yaml"))
        .to_moveit_configs()
    )


    # 1. Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=ns,
        output="screen",
        parameters=[
            # moveit_config.to_dict(),
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics, # <--- 이게 핵심!
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_config.joint_limits,
            {"use_sim_time": True},
            # 아래 파라미터를 수동으로 추가해서 virtual_joint 인식을 돕습니다.
            {"robot_description_kinematics.panda_arm.tip_link": f"{ns}/panda_link8"},
            {"robot_description_kinematics.panda_arm.base_link": f"{ns}/panda_link0"},
            
        ],
    
        arguments=["--ros-args", "--log-level", "info"],
    )

    # 2. RViz2
    rviz_config_file = os.path.join(
        get_package_share_directory("isaac_moveit"),
        "rviz2",
        "panda_moveit_config.rviz", # RViz 설정 파일 내에서도 토픽명 앞에 /franka_1이 붙어있어야 함에 유의
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # namespace=ns,
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": LaunchConfiguration("use_sim_time"),
             "robot_description_kinematics.panda_arm.kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",},
        ],
        remappings=[
        # 로봇 모델과 관절 상태 매칭
            ("/joint_states", f"/{ns}/joint_states"),
            ("/robot_description", f"/{ns}/robot_description"),
            ("/robot_description_semantic", f"/{ns}/robot_description_semantic"),
            # MoveIt 액션 및 서비스 연결 (화살표 소환 핵심)
            ("/move_group", f"/{ns}/move_group"),
            ("/display_planned_path", f"/{ns}/display_planned_path"),
            ("/monitored_planning_scene", f"/{ns}/monitored_planning_scene"),
            ("/get_planning_scene", f"/{ns}/get_planning_scene"),
            ("/planning_scene", f"/{ns}/planning_scene"),
            ("/planning_scene_world", f"/{ns}/planning_scene_world"),
    ]
    )

    # 3. Static TF (World to Robot)
# TF 연결: namespace 인자를 제거하여 전역에서 확실히 world-로봇 연결
    world2robot_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"static_tf_pub_{ns}", # 이름은 겹치지 않게 ns 추가
        output="screen",
        arguments=[
            "0.0", "-0.64", "0.0", "0.0", "0.0", "0.0",
            "world",
            "panda_link0"
        ],
        parameters=[{"use_sim_time": True}], # LaunchConfiguration 대신 True로 강제해서 확인
    )

    # 4. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=ns,
        output="both",
        parameters=[
            moveit_config.robot_description,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                'frame_prefix': "",
                # "frame_prefix": f"{ns}/",
                # 중요: /franka_1/joint_states가 아니라 노드 입장에서의 상대 경로인 'joint_states'를 써야 할 수도 있습니다.
                "qos_overrides./joint_states.subscription.reliability": "reliable",
            },
        ],
    )

    # 5. ROS2 Control Node
    ros2_controllers_path = os.path.join(my_pkg_share, "config", "ros2_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=ns,
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    # 6. Spawners (Controller Manager가 네임스페이스 내에 있으므로 -c 인자 확인)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", f"/{ns}/controller_manager",
        ],
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=["panda_arm_controller", "-c", f"/{ns}/controller_manager"],
    )

    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=["panda_hand_controller", "-c", f"/{ns}/controller_manager"],
    )

    # 7. Custom Panda Control Node
    panda_control_node = Node(
        package="panda_control", 
        executable="main",
        name="panda_control", 
        namespace=ns,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription(
        [
            ros2_control_hardware_type,
            use_sim_time,
            rviz_node,
            world2robot_tf_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            # joint_state_broadcaster_spawner,
            panda_arm_controller_spawner,
            panda_hand_controller_spawner,
            panda_control_node
        ]
    )