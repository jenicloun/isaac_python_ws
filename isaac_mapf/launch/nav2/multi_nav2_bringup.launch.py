from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def make_nav2_group(namespace: str, params_file, map_file, use_sim_time, autostart):
    lifecycle_nodes = [
        "map_server",
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]

    return GroupAction([
        PushRosNamespace(namespace),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": use_sim_time,
                    "yaml_filename": map_file,
                },
            ],
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": lifecycle_nodes,
            }],
        ),
    ])


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
    )

    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
    )

    map_file_arg = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution([
            FindPackageShare("isaac_mapf"),
            "config",
            "carter_warehouse_navigation_005.yaml",
        ]),
    )

    carter1_params_arg = DeclareLaunchArgument(
        "carter1_params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("isaac_mapf"),
            "config",
            "nav2",
            "carter1_nav2_params.yaml",
        ]),
    )

    carter2_params_arg = DeclareLaunchArgument(
        "carter2_params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("isaac_mapf"),
            "config",
            "nav2",
            "carter2_nav2_params.yaml",
        ]),
    )

    carter3_params_arg = DeclareLaunchArgument(
        "carter3_params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("isaac_mapf"),
            "config",
            "nav2",
            "carter3_nav2_params.yaml",
        ]),
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    map_file = LaunchConfiguration("map")

    carter1_params_file = LaunchConfiguration("carter1_params_file")
    carter2_params_file = LaunchConfiguration("carter2_params_file")
    carter3_params_file = LaunchConfiguration("carter3_params_file")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    return LaunchDescription([
        stdout_linebuf_envvar,
        use_sim_time_arg,
        autostart_arg,
        map_file_arg,
        carter1_params_arg,
        carter2_params_arg,
        carter3_params_arg,

        make_nav2_group(
            namespace="carter1",
            params_file=carter1_params_file,
            map_file=map_file,
            use_sim_time=use_sim_time,
            autostart=autostart,
        ),

        make_nav2_group(
            namespace="carter2",
            params_file=carter2_params_file,
            map_file=map_file,
            use_sim_time=use_sim_time,
            autostart=autostart,
        ),

        make_nav2_group(
            namespace="carter3",
            params_file=carter3_params_file,
            map_file=map_file,
            use_sim_time=use_sim_time,
            autostart=autostart,
        ),
    ])