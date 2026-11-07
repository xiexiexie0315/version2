"""Original MJCF-derived robot inspection world."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    share = get_package_share_directory("hnu2027_sentry_simulation")
    models = os.path.join(get_package_share_directory("hnu2027_sentry_description"), "resource", "models")
    return LaunchDescription([
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", models + os.pathsep + os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", models + os.pathsep + os.environ.get("GZ_SIM_RESOURCE_PATH", "")),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "gazebo.launch.py")),
        launch_arguments={"world_sdf_path": os.path.join(share, "resource", "worlds", "original_model_debug.sdf")}.items(),
    )])
