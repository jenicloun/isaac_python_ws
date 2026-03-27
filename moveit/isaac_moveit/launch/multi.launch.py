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
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    
    # moveit_config = (
    #     MoveItConfigsBuilder("moveit_resources_panda")
    #     .robot_description(file_path="config/panda.urdf.xacro")
    #     .robot_description_semantic(file_path="config/panda.srdf")
    #     .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
    #     .planning_pipelines(pipelines=["ompl"])
    #     .to_moveit_configs()
    # )

    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("moveit_resources_panda_moveit_config"),
    #     "config",
    #     "ros2_controllers.yaml",
    # )

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
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        # kinmatics, joint_limits
        .robot_description_kinematics(file_path=os.path.join(config_path, "kinematics.yaml"))
        .joint_limits(file_path=os.path.join(config_path, "joint_limits.yaml"))
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
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
                
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    parameters=[
                        moveit_config.robot_description, 
                        {"use_sim_time": use_sim_time},
                        {"frame_prefix": f"{instance_name}/"},
                    ],
                ),

                Node(
                    package="moveit_ros_move_group",
                    executable="move_group",
                    output="screen",
                    parameters=[
                        moveit_config.to_dict(),
                        {"use_sim_time": use_sim_time},
                        # 플래닝 라이브러리 로드에 필수적인 파라미터들
                        ompl_config,
                        moveit_config.planning_pipelines, 
                        moveit_config.robot_description_kinematics,
                        moveit_config.joint_limits,
                        # 추가: MoveIt이 어떤 컨트롤러를 사용할지(FollowJointTrajectory 등) 알려줌
                        moveit_config.trajectory_execution,
                    ],
                    # RViz가 해당 인스턴스의 서비스를 정확히 찾아오도록 리매핑 확장
                    remappings=[
                        ("/get_planning_scene", f"/{instance_name}/get_planning_scene"),
                        ("/apply_planning_scene", f"/{instance_name}/apply_planning_scene"),
                        ("/execute_trajectory", f"/{instance_name}/execute_trajectory"),
                        ("/move_action", f"/{instance_name}/move_action"),
                        ("/plan_kinematic_path", f"/{instance_name}/plan_kinematic_path"),
                        ("/query_planners", f"/{instance_name}/query_planners"),
                        ("/display_planned_path", f"/{instance_name}/display_planned_path"),
                        ("/query_planner_interface", f"/{instance_name}/query_planner_interface"),
                        
                        # 핵심: Isaac Sim 컨트롤러로 명령을 쏴주는 "액션" 통로 뚫기
                        # MoveIt은 기본적으로 /panda_arm_controller/... 를 호출하려 하므로 
                        # 실제 경로인 /franka_1/panda_arm_controller/... 로 리매핑해줍니다.
                       ("/panda_arm_controller/follow_joint_trajectory", "panda_arm_controller/follow_joint_trajectory"),
                       ("/joint_states", "joint_states"),

                        ("/panda_hand_controller/gripper_cmd", f"/{instance_name}/panda_hand_controller/gripper_cmd"),
                        
                        # 핵심: MoveIt의 실행 명령을 Isaac Sim의 'isaac_joint_commands'로 전달
                        ("/panda_arm_controller/follow_joint_trajectory", f"/{instance_name}/isaac_joint_commands"),
                        
                        # 조인트 상태 피드백 (Isaac Sim이 쏘는 isaac_joint_states를 MoveIt의 joint_states로 연결)
                        ("/joint_states", f"/{instance_name}/isaac_joint_states"),
                    ],
                ),

                # 핵심 수정 1: Controller Manager 설정
                Node(
                    package="controller_manager",
                    executable="ros2_control_node",
                    parameters=[
                        ros2_controllers_path, 
                        {"use_sim_time": use_sim_time},
                        # 핵심: robot_description을 파라미터로 직접 전달
                        {"robot_description": moveit_config.robot_description['robot_description']}
                    ],
                    # remappings는 제거하거나 최소화하고, 서비스 이름을 명시적으로 맞춤
                    remappings=[
                        (f"/{instance_name}/controller_manager/robot_description", f"/{instance_name}/robot_description"),
                    ],
                    output="screen",
                ),

                # 핵심 수정 2: Spawner에 파라미터 직접 명시 (--param-file)
                # 'type'을 못 찾는 에러를 방지하기 위해 설정 파일을 직접 참조하게 함
                # joint state
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "joint_state_broadcaster", 
                        "-c", f"/{instance_name}/controller_manager",
                        # "--param-file", ros2_controllers_path,
                    ],
                    parameters=[{"use_sim_time": use_sim_time}],
                ),
                # panda arm controller
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "panda_arm_controller", 
                        "-c", f"/{instance_name}/controller_manager",
                        # "--param-file", ros2_controllers_path,
                    ],
                    parameters=[{"use_sim_time": use_sim_time}],
                ),
                

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
                            "World",
                            f"{instance_name}/panda_link0"],
                        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                    ),
                # World(대문자)와 world(소문자)를 같은 위치로 강제 연결
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="world_bridge_node",
                    arguments=["0", "0", "0", "0", "0", "0", "World", "world"]
                )    
            ]
        )

    panda_1 = create_panda_group("franka_1", 0.0, -0.64)
    panda_2 = create_panda_group("franka_2", 0.0, 0.64)

    rviz_config_file = os.path.join(
        get_package_share_directory("isaac_moveit"), "rviz2", "panda_moveit_config.rviz"
    )
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.robot_description_kinematics,
    #         moveit_config.planning_pipelines,
    #         moveit_config.joint_limits,
    #         {"use_sim_time": use_sim_time}
    #     ],
    #     arguments=["-d", rviz_config_file],
    # )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {"use_sim_time": True},
            {"move_group_namespace": "/franka_1"}
        ],
        arguments=["-d", rviz_config_file],
        remappings=[
            # 1. 플래닝 씬 관련 (가장 중요)
            ("/monitored_planning_scene", "/franka_1/monitored_planning_scene"),
            ("/get_planning_scene", "/franka_1/get_planning_scene"),
            
            # 2. 플래닝 인터페이스 관련 (이게 맞아야 Library가 뜸)
            ("/plan_kinematic_path", "/franka_1/plan_kinematic_path"),
            ("/query_planner_interface", "/franka_1/query_planner_interface"), # 서비스 리스트에 있는 이름 확인됨
            ("/get_planner_params", "/franka_1/get_planner_params"),
            
            # 3. 액션 및 기타 (실행을 위해 필요)
            ("/move_action", "/franka_1/move_action"),
            ("/execute_trajectory", "/franka_1/execute_trajectory"),
            ("/robot_description", "/franka_1/robot_description"),
            ("/robot_description_semantic", "/franka_1/robot_description_semantic"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        panda_1,
        panda_2,
        rviz_node
    ])