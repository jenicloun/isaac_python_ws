import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def load_yaml(file_path):
    try:
        with open(file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():    


        # Command-line arguments
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="isaac",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    # Declare use_sim_time argument
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true",
    )


    config_path = os.path.join(
        get_package_share_directory("isaac_moveit"), "config"
    )

    # 사용자 패키지의 OMPL 파일 경로
    my_ompl_yaml_path = os.path.join(
        get_package_share_directory("isaac_moveit"), "config", "ompl_planning.yaml"
    )
    my_ompl_config = load_yaml(my_ompl_yaml_path)

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
                            file_path="config/panda.urdf.xacro",
                            mappings={
                                "ros2_control_hardware_type": LaunchConfiguration(
                                    "ros2_control_hardware_type"), 
                            }
                        )
        .robot_description_semantic(file_path="config/panda.srdf")
        # kinmatics, joint_limits
        .robot_description_kinematics(file_path=os.path.join(config_path, "kinematics.yaml"))
        .joint_limits(file_path=os.path.join(config_path, "joint_limits.yaml"))
        # .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .trajectory_execution(file_path=os.path.join(get_package_share_directory("isaac_moveit"), "config", "gripper_moveit_controllers.yaml"))
        .planning_pipelines(pipelines=["ompl"])
        # 아래 한 줄을 추가하여 MoveIt이 대문자 World를 기준으로 생각하게 만듭니다.
        .planning_scene_monitor(publish_planning_scene=True, publish_geometry_updates=True, publish_state_updates=True, publish_transforms_updates=True)
        .to_moveit_configs()
    )

    # 핵심: Builder가 로드한 ompl 설정을 사용자 파일 내용으로 통째로 교체
    # YAML 파일 구조상 ['/**']['ros__parameters'] 안에 실제 내용이 있으므로 이를 추출합니다.
    if my_ompl_config and '/**' in my_ompl_config:
        moveit_config.planning_pipelines["ompl"] = my_ompl_config['/**']['ros__parameters']


    ros2_controllers_path = os.path.join(
        get_package_share_directory("isaac_moveit"),
        "config",
        "ros2_controllers.yaml",
    )

    def create_panda_group(instance_name, x_pos, y_pos):

        # 1. YAML 파일 읽기
        ompl_yaml_path = os.path.join(
            get_package_share_directory("isaac_moveit"), "config", "ompl_planning.yaml"
        )
        with open(ompl_yaml_path, 'r') as f:
            # safe_load로 읽은 후 [/**][ros__parameters] 계층을 걷어내고 순수 설정만 가져옴
            ompl_config = yaml.safe_load(f)['/**']['ros__parameters']

        return GroupAction(
            actions=[
                PushRosNamespace(instance_name),
                
                # Input: Joint movement Output: tf
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name="robot_state_publisher",
                    output="both",
                    parameters=[
                        moveit_config.robot_description, 
                        {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    ],
                    
                remappings=[
                    ("joint_states", "isaac_joint_states"),
                    # 중요: TF 출력을 네임스페이스 없는 전역 /tf로 강제 리매핑
                    ("/tf", "/tf"),
                    ("/tf_static", "/tf_static")
                ]
                ),

                Node(
                    package="moveit_ros_move_group",
                    executable="move_group",
                    output="screen",
                    parameters=[
                        moveit_config.to_dict(),
                        {"use_sim_time": LaunchConfiguration("use_sim_time")},
                        ompl_config,
                        moveit_config.planning_pipelines, 
                        moveit_config.robot_description_kinematics,
                        moveit_config.joint_limits,
                        moveit_config.trajectory_execution,
                    ],
                    remappings=[
                        ("robot_description", "robot_description"),
                        ("robot_description_semantic", "robot_description_semantic"),
                        # Planning Scene도 네임스페이스 안의 것을 보도록 명시
                        # ("panda_hand_controller/gripper_cmd", f"/{instance_name}/panda_hand_controller/gripper_cmd"),
                        # 핵심: MoveIt의 실행 명령을 Isaac Sim의 'isaac_joint_commands'로 전달
                        ("panda_arm_controller/follow_joint_trajectory", f"/{instance_name}/panda_arm_controller/follow_joint_trajectory"),
                        ("joint_states", f"/{instance_name}/joint_states"),
                    ],
                    arguments=["--ros-args", "--log-level", "info"],
                ),

                # 핵심 수정 1: Controller Manager 설정
                Node(
                    package="controller_manager",
                    executable="ros2_control_node",
                    parameters=[
                        ros2_controllers_path, 
                        {"use_sim_time": LaunchConfiguration("use_sim_time")},
                        # 핵심: robot_description을 파라미터로 직접 전달
                        {"robot_description": moveit_config.robot_description['robot_description']}
                    ],
                    # remappings는 제거하거나 최소화하고, 서비스 이름을 명시적으로 맞춤
                    remappings=[
                        (f"/{instance_name}/controller_manager/robot_description", f"/{instance_name}/robot_description"),
                    ],
                    output="screen",
                ),

                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "joint_state_broadcaster", 
                        "--controller-manager", 
                        f"/{instance_name}/controller_manager",
                    ]
                    # parameters=[{"use_sim_time": use_sim_time}],
                ),
                # panda arm controller
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "panda_arm_controller", "-c", 
                        f"/{instance_name}/controller_manager",
                    ],
                    # parameters=[{"use_sim_time": use_sim_time}],
                ),

                # Input: x, y,z, roll, pitch, yaw, (fixed position) Output: /tf
                Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="static_transform_publisher_world_to_robot",
                        output="log",
                        arguments=[
                            str(x_pos),
                            str(y_pos),
                            "0.0",
                            "0.0",
                            "0.0",
                            "0.0",
                            "world",
                            "panda_link0"],
                        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                    ),
                # World(대문자)와 world(소문자)를 같은 위치로 강제 연결
                # Node(
                #     package="tf2_ros",
                #     executable="static_transform_publisher",
                #     name="world_bridge_node",
                #     arguments=["0", "0", "0", "0", "0", "0", "World", "world"]
                # )    
            ]
        )

    panda_1 = create_panda_group("franka_1", 0.0, -0.64)
    # panda_2 = create_panda_group("franka_2", 0.0, 0.64)

    rviz_config_file = os.path.join(
        get_package_share_directory("isaac_moveit"), "rviz2", "panda_moveit_config.rviz"
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
            {"franka_1.robot_description_kinematics": moveit_config.robot_description_kinematics["robot_description_kinematics"]},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            
        ],
        remappings=[
            # RViz가 기본적으로 찾는 이름을 franka_1 아래에 있는 것으로 연결
            ("/robot_description", "/franka_1/robot_description"),
            ("/robot_description_semantic", "/franka_1/robot_description_semantic"),
            # (중요) 플래닝 씬과 경로 표시도 연결
            ("/monitored_planning_scene", "/franka_1/monitored_planning_scene"),
            ("/display_planned_path", "/franka_1/display_planned_path"),
            ("/get_planning_scene", "/franka_1/get_planning_scene"), 
            ("/planning_scene", "/franka_1/planning_scene"),         
        ],
    )

    return LaunchDescription([
        ros2_control_hardware_type,
        use_sim_time,
        panda_1,
        rviz_node
    ])