"""Visible joint/coordination simulation; does not start a real transport."""
import os
import tempfile
from ament_index_python.packages import get_package_share_directory as share, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from hnu_gazebo_adapter.world_builder import build_world

def start(context):
    adapter_share = share("hnu_gazebo_adapter")
    simulation_share = share("hnu2027_sentry_simulation")
    folder = tempfile.mkdtemp(prefix="hnu_coordinator_world_")
    world = build_world(share("hnu2027_sentry_description"), simulation_share,
                        os.path.join(folder, "coordinator_sim.sdf"))
    model = "/model/hnu2027_sentry"
    args = [
        model + "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
        model + "/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        model + "/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
    ]
    for joint in ("big_yaw", "small_yaw", "pitch", "fold_roll"):
        args.append(model + "/joint/" + joint + "/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double")
    return [
        SetEnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
            os.path.join(get_package_prefix("hnu_gazebo_adapter"), "lib") + os.pathsep +
            os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "")),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(simulation_share, "launch/gazebo.launch.py")),
            launch_arguments={"world_sdf_path": world,
                              "gui": LaunchConfiguration("gui").perform(context)}.items()),
        Node(package="ros_gz_bridge", executable="parameter_bridge", arguments=args,
             parameters=[{"use_sim_time": True}], output="screen"),
        Node(package="hnu_tunnel_controller", executable="navigation_coordinator_node",
             name="navigation_coordinator", output="screen",
             parameters=[os.path.join(share("hnu2027_sentry_bringup"),
                         "config/navigation_coordinator.yaml"), {"use_sim_time": True}]),
        Node(package="hnu_gazebo_adapter", executable="gazebo_controller_adapter",
             name="gazebo_controller_adapter", output="screen",
             parameters=[os.path.join(adapter_share, "config/gazebo_adapter.yaml"),
                         {"use_sim_time": True}]),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("simulation_domain_id", default_value="181"),
        DeclareLaunchArgument("ign_partition", default_value="hnu_coordination_sim"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("simulation_domain_id")),
        SetEnvironmentVariable("IGN_PARTITION", LaunchConfiguration("ign_partition")),
        OpaqueFunction(function=start)])
