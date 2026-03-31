import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 설정: 네임스페이스 변수
    ns = "franka_1"
    prefix = "franka_1_"

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


    my_pkg_share = get_package_share_directory("isaac_moveit")


    moveit_config = (
    MoveItConfigsBuilder(f"{prefix}", package_name="isaac_moveit")
    .robot_description(
        file_path=os.path.join(my_pkg_share, "config", "panda.urdf.xacro"),
        mappings={
            "ros2_control_hardware_type": LaunchConfiguration("ros2_control_hardware_type"),
            "prefix": prefix,
        },
    )
    .robot_description_semantic(
        file_path=os.path.join(my_pkg_share, "config", "panda.srdf"),
        mappings={"prefix": prefix} 
    )
        # ... 나머지 설정 동일
        .robot_description_kinematics(file_path=os.path.join(my_pkg_share, "config", "franka_1_kinematics.yaml"))
        .trajectory_execution(file_path=os.path.join(my_pkg_share, "config", "franka_1_gripper_moveit_controllers.yaml"))
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path=os.path.join(my_pkg_share, "config", "franka_1_joint_limits.yaml"))
        .to_moveit_configs()
    )


# 1️⃣ JointState Remapper Node
    # --------------------------
    joint_remapper_node = Node(
    package="isaac_moveit",
    executable="joint_state_remapper",  # setup.py entry_point 이름과 동일
    name="joint_state_remapper",
    namespace=ns,
    output="screen",
    )

    # 1. Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=ns,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,  # 반드시 추가
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_model_root": f"{prefix}panda_link0",
                "publish_robot_description_semantic": True,
                "planning_pipelines": ["ompl"],
                "default_planning_pipeline": "ompl",
            },
        ],
        # move_group_node 설정 예시
        remappings=[
            # ("/joint_states", f"/{ns}/joint_states"),
            ("/joint_states", "/franka_1_isaac_joint_states"),
            ("/display_planned_path", f"/{ns}/display_planned_path"),
            ("/tf", "/tf"),
            ("/tf_static", "/tf_static"),
            # MoveIt이 내부적으로 사용하는 토픽들을 네임스페이스로 강제 연결
            # ("robot_description", f"/{ns}/robot_description"),
            # ("robot_description_semantic", f"/{ns}/robot_description_semantic"),
        ],
    )

    # 2. RViz2
    rviz_config_file = os.path.join(
        get_package_share_directory("isaac_moveit"),
        "rviz2",
        "panda_moveit_config_sj.rviz", # RViz 설정 파일 내에서도 토픽명 앞에 /franka_1이 붙어있어야 함에 유의
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": LaunchConfiguration("use_sim_time"),
             "robot_description_kinematics.panda_arm.kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin"},
        ],
        remappings=[
        # 로봇 모델과 관절 상태 매칭
            # ("/joint_states", f"/{ns}/joint_states"),
            ("/joint_states", "/franka_1_isaac_joint_states"),
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
    world2robot_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "0.0", "-0.64", "0.0", "0.0", "0.0", "0.0",
            "world",
            f"{prefix}panda_link0"
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}], # LaunchConfiguration 대신 True로 강제해서 확인
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
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
        ("/joint_states", "/franka_1_isaac_joint_states")
    ]
    )

    # 5. ROS2 Control Node
    ros2_controllers_path = os.path.join(my_pkg_share, "config", "franka_1_ros2_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=ns,
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        # remappings=[
        #     ("/controller_manager/robot_description", f"/{ns}/robot_description"),
        # ],
        output="screen",
    )

    # 6. Spawners (Controller Manager가 네임스페이스 내에 있으므로 -c 인자 확인)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
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
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        
    )

    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=["panda_hand_controller", "-c", f"/{ns}/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
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
            # joint_remapper_node,
            rviz_node,
            world2robot_tf_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            panda_arm_controller_spawner,
            panda_hand_controller_spawner,
            # panda_control_node
        ]
    )