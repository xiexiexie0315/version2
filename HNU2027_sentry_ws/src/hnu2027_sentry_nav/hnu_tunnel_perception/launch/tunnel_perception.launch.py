"""Launch a read-only tunnel detector; no control nodes are started."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("input_cloud_topic", default_value="registered_scan"),
        DeclareLaunchArgument("tunnel_id", default_value="blue_spawn_tunnel"),
        DeclareLaunchArgument("center_x_m", default_value="-2.50"),
        DeclareLaunchArgument("center_y_m", default_value="3.50"),
        DeclareLaunchArgument("axis_yaw_rad", default_value="0.0"),
        DeclareLaunchArgument("floor_elevation_m", default_value="0.1005"),
        Node(
            package="hnu_tunnel_perception",
            executable="tunnel_detector_node",
            name="tunnel_detector",
            output="screen",
            parameters=[{
                "input_cloud_topic": LaunchConfiguration("input_cloud_topic"),
                "tunnel_id": LaunchConfiguration("tunnel_id"),
                "center_x_m": LaunchConfiguration("center_x_m"),
                "center_y_m": LaunchConfiguration("center_y_m"),
                "axis_yaw_rad": LaunchConfiguration("axis_yaw_rad"),
                "floor_elevation_m": LaunchConfiguration("floor_elevation_m"),
            }],
        ),
    ])
