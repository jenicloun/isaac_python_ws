# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node, PushRosNamespace
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     namespace_arg = DeclareLaunchArgument(
#         "namespace",
#         default_value="carter1",
#     )

#     use_sim_time_arg = DeclareLaunchArgument(
#         "use_sim_time",
#         default_value="true",
#     )

#     autostart_arg = DeclareLaunchArgument(
#         "autostart",
#         default_value="true",
#     )

#     params_file_arg = DeclareLaunchArgument(
#         "params_file",
#         default_value=PathJoinSubstitution([
#             FindPackageShare("isaac_mapf"),
#             "config",
#             "nav2",
#             "carter1_nav2_params.yaml",
#         ]),
#     )

#     namespace = LaunchConfiguration("namespace")
#     use_sim_time = LaunchConfiguration("use_sim_time")
#     autostart = LaunchConfiguration("autostart")
#     params_file = LaunchConfiguration("params_file")

#     lifecycle_nodes = [
#         "controller_server",
#         "planner_server",
#         "behavior_server",
#         "bt_navigator",
#     ]

#     stdout_linebuf_envvar = SetEnvironmentVariable(
#         "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
#     )

#     remappings = [
#         ('map', '/mapf/map'),
#     ]

#     return LaunchDescription([
#         stdout_linebuf_envvar,
#         namespace_arg,
#         use_sim_time_arg,
#         autostart_arg,
#         params_file_arg,

#         GroupAction([
#             PushRosNamespace(namespace),

#             Node(
#                 package="nav2_controller",
#                 executable="controller_server",
#                 name="controller_server",
#                 output="screen",
#                 parameters=[params_file, {"use_sim_time": use_sim_time}],
#                 remappings=remappings,
#             ),
#             Node(
#                 package="nav2_planner",
#                 executable="planner_server",
#                 name="planner_server",
#                 output="screen",
#                 parameters=[params_file, {"use_sim_time": use_sim_time}],
#                 remappings=remappings,
#             ),
#             Node(
#                 package="nav2_behaviors",
#                 executable="behavior_server",
#                 name="behavior_server",
#                 output="screen",
#                 parameters=[params_file, {"use_sim_time": use_sim_time}],
#                 remappings=remappings,
#             ),
#             Node(
#                 package="nav2_bt_navigator",
#                 executable="bt_navigator",
#                 name="bt_navigator",
#                 output="screen",
#                 parameters=[params_file, {"use_sim_time": use_sim_time}],
#                 remappings=remappings,
#             ),
#             Node(
#                 package="nav2_lifecycle_manager",
#                 executable="lifecycle_manager",
#                 name="lifecycle_manager_navigation",
#                 output="screen",
#                 parameters=[{
#                     "use_sim_time": use_sim_time,
#                     "autostart": autostart,
#                     "node_names": lifecycle_nodes,
#                 }],
#             ),
#         ]),
#     ])


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="carter1",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
    )

    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("isaac_mapf"),
            "config",
            "nav2",
            "carter1_nav2_params.yaml",
        ]),
    )

    map_file_arg = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution([
            FindPackageShare("isaac_mapf"),
            "config",
            "carter_warehouse_navigation_005.yaml",
        ]),
    )

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")

    lifecycle_nodes = [
        "map_server",
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    return LaunchDescription([
        stdout_linebuf_envvar,
        namespace_arg,
        use_sim_time_arg,
        autostart_arg,
        params_file_arg,
        map_file_arg,

        GroupAction([
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
                    }
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
        ]),
    ])