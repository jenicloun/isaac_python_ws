from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import math


def pub_initialpose(topic: str, x: float, y: float, yaw: float = 0.0):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)

    msg = f"""
header:
  frame_id: map
pose:
  pose:
    position: {{x: {x}, y: {y}, z: 0.0}}
    orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.068]
"""
    return ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--once",
            topic,
            "geometry_msgs/msg/PoseWithCovarianceStamped",
            msg,
        ],
        output="screen",
    )
    
def robot_group(robot_name: str, map_file, use_sim_time, autostart, amcl_params_file):
    localization_nodes = ["amcl"]

    return GroupAction([
        PushRosNamespace(robot_name),

        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[
                amcl_params_file,
                {"use_sim_time": use_sim_time},
            ],
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": localization_nodes,
            }],
        ),
    ])


def generate_launch_description():
    map_file_arg = DeclareLaunchArgument(
        name="map",
        default_value="carter_warehouse_navigation_005.yaml",
        description="YAML map file name",
    )
    mapf_map_file_arg = DeclareLaunchArgument(
        name="mapf_map",
        default_value="carter_warehouse_navigation.yaml", # resolution 1.5 맵
        description="Low resolution YAML map for MAPF",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation clock if true",
    )
    autostart_arg = DeclareLaunchArgument(
        name="autostart",
        default_value="true",
        description="Automatically startup lifecycle nodes",
    )
    map_file = PathJoinSubstitution([
        FindPackageShare("isaac_mapf"),
        "config",
        LaunchConfiguration("map"),
    ])
    mapf_map_file = PathJoinSubstitution([
        FindPackageShare("isaac_mapf"), "config", LaunchConfiguration("mapf_map"),
    ])

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    mapf_lifecycle_nodes = ["map_server", "mapf_base_node"]

    mapf_params = PathJoinSubstitution([
        FindPackageShare("isaac_mapf"),
        "config",
        "mapf_params_3g.yaml",
    ])

    costmap_params = PathJoinSubstitution([
        FindPackageShare("isaac_mapf"),
        "config",
        "costmap_params.yaml",
    ])

    carter1_amcl_params = PathJoinSubstitution([
        FindPackageShare("isaac_mapf"),
        "config",
        "amcl",
        "carter1_amcl_params.yaml",
    ])

    carter2_amcl_params = PathJoinSubstitution([
        FindPackageShare("isaac_mapf"),
        "config",
        "amcl",
        "carter2_amcl_params.yaml",
    ])

    carter3_amcl_params = PathJoinSubstitution([
        FindPackageShare("isaac_mapf"),
        "config",
        "amcl",
        "carter3_amcl_params.yaml",
    ])

    return LaunchDescription([
        map_file_arg,
        mapf_map_file_arg,  
        use_sim_time_arg,
        autostart_arg,

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                {"yaml_filename": map_file},
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["map_server"],
            }],
        ),

        # -----------------------------
        # TF relay
        # -----------------------------
        Node(
            package="isaac_mapf",
            executable="tf_prefix_relay",
            name="carter1_tf_relay",
            output="screen",
            parameters=[
                {"input_tf_topic": "/carter1/tf"},
                {"output_tf_topic": "/tf"},
                {"prefix": "carter1"},
                {"global_frames": ["World", "map", "odom"]},
            ],
        ),
        Node(
            package="isaac_mapf",
            executable="tf_prefix_relay",
            name="carter2_tf_relay",
            output="screen",
            parameters=[
                {"input_tf_topic": "/carter2/tf"},
                {"output_tf_topic": "/tf"},
                {"prefix": "carter2"},
                {"global_frames": ["World", "map", "odom"]},
            ],
        ),
        Node(
            package="isaac_mapf",
            executable="tf_prefix_relay",
            name="carter3_tf_relay",
            output="screen",
            parameters=[
                {"input_tf_topic": "/carter3/tf"},
                {"output_tf_topic": "/tf"},
                {"prefix": "carter3"},
                {"global_frames": ["World", "map", "odom"]},
            ],
        ),

        # -----------------------------
        # AMCL localization per robot
        # -----------------------------
        robot_group("carter1", map_file, use_sim_time, autostart, carter1_amcl_params),
        robot_group("carter2", map_file, use_sim_time, autostart, carter2_amcl_params),
        robot_group("carter3", map_file, use_sim_time, autostart, carter3_amcl_params),

        
        
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg="[amcl_init] Publishing initial poses"),
                pub_initialpose("/carter1/initialpose", -5.0, -5.0, 0.0),
                pub_initialpose("/carter2/initialpose", -5.0, -7.0, 0.0),
                pub_initialpose("/carter3/initialpose", -5.0, -9.0, 0.0),
            ],
        ),
        # -----------------------------
        # plan_to_cmdvel
        # -----------------------------
        Node(
            package='mapf_base',
            executable='plan_executor',
            name='plan_executor',
            output='screen',
            # ==== [여기 추가!] 토픽 이름 맞춰주기 ====
            remappings=[
                ('global_plan', '/mapf/global_plan') 
            ],
            # ========================================
            parameters=[{
                'agent_num': 3,
                'global_frame_id': 'map',
                'xy_goal_tolerance': 0.01,
                'yaw_goal_tolerance': 0.01,
                
                'agent_name.agent_0': 'carter1',
                'base_frame_id.agent_0': 'carter1/chassis_link/base_link',
                'plan_topic.agent_0': '/mapf/global_plan', # (C++에서 안 쓰지만 구색 맞추기용)
                
                'agent_name.agent_1': 'carter2',
                'base_frame_id.agent_1': 'carter2/chassis_link/base_link',
                'plan_topic.agent_1': '/mapf/global_plan',
                
                'agent_name.agent_2': 'carter3',
                'base_frame_id.agent_2': 'carter3/chassis_link/base_link',
                'plan_topic.agent_2': '/mapf/global_plan',
            }]
        ),

        # -----------------------------
        # MAPF side
        # -----------------------------
        GroupAction([
            PushRosNamespace("mapf"),

            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                respawn=True,
                parameters=[
                    mapf_params,
                    {"yaml_filename": mapf_map_file},
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="mapf_base",
                executable="mapf_base_node",
                name="mapf_base_node",
                output="screen",
                respawn=True,
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
                    {"node_names": mapf_lifecycle_nodes},
                ],
            ),
        ]),

        TimerAction(
            period=2.0,
            actions=[
                GroupAction([
                    PushRosNamespace("mapf"),
                    Node(
                        package="mapf_base",
                        executable="goal_transformer",
                        name="goal_transformer",
                        output="screen",
                        parameters=[mapf_params, {"use_sim_time": use_sim_time}],
                    ),
                ])
            ],
        ),
    ])