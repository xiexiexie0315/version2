"""Safe stage-seven tunnel integration entry point.

It may start the 2027 simulation world, but does not enable command takeover.
The action server and Nav2 command wiring are intentionally separate launch
arguments so a missing subsystem never makes a physical robot move.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sim_share = get_package_share_directory("hnu2027_sentry_simulation")
    start_sim = LaunchConfiguration("start_sim")
    enable_control = LaunchConfiguration("enable_control")
    return LaunchDescription([
        DeclareLaunchArgument("start_sim", default_value="true"),
        DeclareLaunchArgument("enable_control", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_share, "launch", "gazebo.launch.py")),
            condition=IfCondition(start_sim),
        ),
        LogInfo(msg=["HNU 2027 tunnel full-stack scaffold. enable_control=", enable_control,
                     ". Control takeover remains disabled until Action server and cmd_vel wiring are verified."]),
    ])
