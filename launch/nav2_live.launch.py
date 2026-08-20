from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    static_map = LaunchConfiguration("static_map")
    map_yaml = LaunchConfiguration("map_yaml")
    occupancy_map_topic = LaunchConfiguration("occupancy_map_topic")
    occupancy_save = LaunchConfiguration("occupancy_save")
    occupancy_save_directory = LaunchConfiguration("occupancy_save_directory")
    occupancy_save_name = LaunchConfiguration("occupancy_save_name")
    common_parameters = [params_file, {"use_sim_time": use_sim_time}]

    managed_nodes = [
        "controller_server",
        "velocity_smoother",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("static_map", default_value="false"),
            DeclareLaunchArgument("map_yaml", default_value=""),
            DeclareLaunchArgument("occupancy_map_topic", default_value="/map"),
            DeclareLaunchArgument("occupancy_save", default_value="false"),
            DeclareLaunchArgument("occupancy_save_directory", default_value=""),
            DeclareLaunchArgument("occupancy_save_name", default_value="map"),
            Node(
                package="autonomy_light",
                executable="live_occupancy_mapper",
                name="live_occupancy_mapper",
                output="screen",
                parameters=[
                    *common_parameters,
                    {
                        "map_topic": occupancy_map_topic,
                        "save_map": occupancy_save,
                        "save_map_directory": occupancy_save_directory,
                        "save_map_name": occupancy_save_name,
                    },
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                condition=IfCondition(static_map),
                parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
            ),
            Node(
                package="autonomy_light",
                executable="command_user_bridge",
                name="command_user_bridge",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=common_parameters,
                remappings=[("cmd_vel", "/nav2/cmd_vel_raw")],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=common_parameters,
                remappings=[
                    ("cmd_vel", "/nav2/cmd_vel_raw"),
                    ("cmd_vel_smoothed", "/nav2/cmd_vel"),
                ],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=common_parameters,
                remappings=[("cmd_vel", "/nav2/cmd_vel_raw")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "bond_timeout": 4.0,
                        "attempt_respawn_reconnection": True,
                        "node_names": managed_nodes,
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map_server",
                output="screen",
                condition=IfCondition(static_map),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "bond_timeout": 4.0,
                        "attempt_respawn_reconnection": True,
                        "node_names": ["map_server"],
                    }
                ],
            ),
        ]
    )
