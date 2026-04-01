import math
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, TimerAction, 
                            ExecuteProcess, LogInfo, SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

# --- 헬퍼 함수 정의 ---

def pub_initialpose(topic: str, x: float, y: float, yaw: float = 0.0):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    msg = f"{{header: {{frame_id: 'map'}}, pose: {{pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.068]}}}}"
    return ExecuteProcess(cmd=["ros2", "topic", "pub", "--once", topic, "geometry_msgs/msg/PoseWithCovarianceStamped", msg], output="screen")

def pub_goal(topic: str, x: float, y: float, yaw: float = 0.0):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    msg = f"{{header: {{frame_id: 'map'}}, pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}}}}}"
    return ExecuteProcess(cmd=['ros2', 'topic', 'pub', '--once', topic, 'geometry_msgs/msg/PoseStamped', msg], output='screen')

def make_nav2_group(namespace: str, params_file, map_file, use_sim_time, autostart):
    # 각 노드별로 정확한 패키지 이름을 매핑합니다.
    nav2_nodes = [
        ("nav2_map_server", "map_server"),
        ("nav2_controller", "controller_server"),
        ("nav2_planner", "planner_server"),
        ("nav2_behaviors", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator"),
    ]

    return GroupAction([
        PushRosNamespace(namespace),

        # 노드 생성 루프
        *[Node(
            package=pkg,
            executable=exe,
            name=exe,
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": use_sim_time, "yaml_filename": map_file} if exe == "map_server" else {"use_sim_time": use_sim_time}
            ]
        ) for pkg, exe in nav2_nodes],

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_nav",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": [n[1] for n in nav2_nodes], # exe 이름들만 추출
            }],
        ),
    ])

def generate_launch_description():
    # 1. 환경 설정 및 인자
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    autostart = LaunchConfiguration("autostart", default="true")
    pkg_share = FindPackageShare("isaac_mapf")

    # 2. 공통 요소
    map_file = PathJoinSubstitution([pkg_share, "config", "carter_warehouse_navigation_005.yaml"])
    mapf_map_file = PathJoinSubstitution([pkg_share, "config", "carter_warehouse_navigation.yaml"])


    mapf_params = PathJoinSubstitution([
    FindPackageShare("isaac_mapf"), "config", "mapf_params_3g.yaml"])

    costmap_params = PathJoinSubstitution([
        FindPackageShare("isaac_mapf"), "config", "costmap_params.yaml"])
    
    # 3. 로봇별 네비게이션 및 AMCL 설정 (하나의 그룹으로 묶음)
    robot_names = ["carter1", "carter2", "carter3"]
    nav_nodes = []
    for name in robot_names:
        params = PathJoinSubstitution([pkg_share, "config", "nav2", f"{name}_nav2_params.yaml"])
        amcl_params = PathJoinSubstitution([pkg_share, "config", "amcl", f"{name}_amcl_params.yaml"])
        
        # Nav2 Stack
        nav_nodes.append(make_nav2_group(name, params, map_file, use_sim_time, autostart))
        # AMCL & Lifecycle
        nav_nodes.append(GroupAction([
            PushRosNamespace(name),
            Node(package="nav2_amcl", executable="amcl", name="amcl", parameters=[amcl_params, {"use_sim_time": use_sim_time}]),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lc_amcl", parameters=[{"use_sim_time": use_sim_time, "autostart": autostart, "node_names": ["amcl"]}])
        ]))
        # TF Relay
        nav_nodes.append(Node(package="isaac_mapf", executable="tf_prefix_relay", name=f"{name}_tf_relay", 
                             parameters=[{"input_tf_topic": f"/{name}/tf", "output_tf_topic": "/tf", "prefix": name, "global_frames": ["World", "map", "odom"]}]))

    # --- 메인 런치 설명 구성 ---
    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),

        # A. 외부 실행하던 명령어들 (Static TF & RViz)
        Node(package="tf2_ros", executable="static_transform_publisher", name="world_to_map", arguments=["5", "5", "0", "0", "0", "0", "World", "map"], parameters=[{"use_sim_time": use_sim_time}]),
        Node(package="rviz2", executable="rviz2", name="rviz2", parameters=[{"use_sim_time": use_sim_time}]),

        # B. 글로벌 맵 서버 (MAPF용)
        Node(package="nav2_map_server", executable="map_server", name="global_map_server", parameters=[{"yaml_filename": map_file, "use_sim_time": use_sim_time}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lc_map", parameters=[{"use_sim_time": use_sim_time, "autostart": autostart, "node_names": ["global_map_server"]}]),

        # C. 로봇별 노드들 (Nav2, AMCL, Relay)
        *nav_nodes,

        # D. MAPF 관련 (Namespace: mapf)
        GroupAction([
            PushRosNamespace("mapf"),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="mapf_map_server",  # 1. 이름을 'mapf_map_server'로 수정
                output="screen",
                parameters=[
                    mapf_params,
                    {"yaml_filename": mapf_map_file, "use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="mapf_base",
                executable="mapf_base_node",
                name="mapf_base_node",   # 2. 이 이름과
                output="screen",
                parameters=[
                    costmap_params,
                    mapf_params,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_mapf",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    # 3. 아래 리스트의 이름들을 위 name들과 정확히 일치시킴
                    {"node_names": ["mapf_map_server", "mapf_base_node"]},
                ],
            ),
        ]),

    #     # E. 자동화 액션 (Timer)
    #     # 초기 포즈 설정
    #     TimerAction(period=5.0, actions=[
    #         pub_initialpose("/carter1/initialpose", -5.0, -9.0, 90.0),
    #         pub_initialpose("/carter2/initialpose",  0.0, -9.0, 90.0),
    #         pub_initialpose("/carter3/initialpose",  5.0, -9.0, 90.0),
    #     ]),
    #     # 초기 목표 전송 및 Flag
    #     TimerAction(period=8.0, actions=[
    #         pub_goal('/mapf/carter1/goal', 5.0, 5.0, 0.0),
    #         pub_goal('/mapf/carter2/goal', -2.475, -4.475, 0.0),
    #         pub_goal('/mapf/carter3/goal', -0.975, -2.975, 0.0),
    #         ExecuteProcess(cmd=['ros2', 'topic', 'pub', '--once', '/mapf/goal_init_flag', 'std_msgs/msg/Bool', '{data: true}'], output='screen')
    #     ]),
    ])