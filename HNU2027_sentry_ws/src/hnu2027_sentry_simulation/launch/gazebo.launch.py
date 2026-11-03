"""Launch only the HNU 2027 Gazebo world and /clock bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("hnu2027_sentry_simulation")
    world = LaunchConfiguration("world_sdf_path")
    models = os.path.join(share, "resource", "models")
    return LaunchDescription([
        DeclareLaunchArgument(
            "world_sdf_path",
            default_value=os.path.join(share, "resource", "worlds", "rmul_2027_3v3_world.sdf"),
        ),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            [EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""), os.pathsep, models],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": ["-r ", world]}.items(),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),
    ])
