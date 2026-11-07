"""Stage one: only coordinator, no transport or actuator nodes."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("hnu2027_sentry_bringup"),
        "config", "navigation_coordinator.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="hnu_tunnel_controller",
             executable="navigation_coordinator_node",
             name="navigation_coordinator", output="screen",
             parameters=[LaunchConfiguration("params_file"),
                         {"use_sim_time": LaunchConfiguration("use_sim_time")}]),
    ])
