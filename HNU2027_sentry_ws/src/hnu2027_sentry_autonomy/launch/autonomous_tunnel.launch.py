"""Real scan + AMCL + Nav2 + HNU workflow in the original Gazebo field."""
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory as share, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from hnu2027_sentry_autonomy.world_builder import build_world

def start(context):
    directory=tempfile.mkdtemp(prefix="hnu_autonomy_")
    own=share("hnu2027_sentry_autonomy")
    sim=share("hnu2027_sentry_simulation")
    lidar_geometry=LaunchConfiguration("lidar_geometry").perform(context)
    if lidar_geometry not in ("collision","visual"):
        raise RuntimeError("lidar_geometry must be collision or visual")
    world=build_world(share("hnu2027_sentry_description"),sim,os.path.join(directory,"world.sdf"),
        block_tunnel=LaunchConfiguration("block_tunnel").perform(context).lower()=="true",
        lidar_collision_geometry=lidar_geometry=="collision")
    nav=yaml.safe_load(open(os.path.join(own,"config/nav2.yaml")))
    nav["map_server"]["ros__parameters"]["yaml_filename"]=os.path.join(
        share("hnu_tunnel_bringup"),"map/simulation/rmul_2027_3v3.yaml")
    nav["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"]=os.path.join(own,"config/navigate.xml")
    nav["bt_navigator"]["ros__parameters"]["default_nav_through_poses_bt_xml"]=os.path.join(own,"config/navigate_through.xml")
    params=os.path.join(directory,"nav2.yaml")
    with open(params,"w") as f:yaml.safe_dump(nav,f,sort_keys=False)
    model="/model/hnu2027_sentry"
    bridges=[
      model+"/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
      model+"/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
      model+"/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
      "/simulation/raw_scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
      "/simulation/raw_tunnel_scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked"]
    for joint in ("big_yaw","small_yaw","pitch","fold_roll"):
        bridges.append(model+"/joint/"+joint+"/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double")
    nodes=[
      SetEnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
        os.path.join(get_package_prefix("hnu_gazebo_adapter"),"lib")+os.pathsep+
        os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH","")),
      IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(sim,"launch/gazebo.launch.py")),
        launch_arguments={"world_sdf_path":world,"gui":LaunchConfiguration("gui").perform(context)}.items()),
      Node(package="ros_gz_bridge",executable="parameter_bridge",arguments=bridges,output="screen",
           parameters=[{"use_sim_time":True}]),
      Node(package="hnu_gazebo_adapter",executable="gazebo_controller_adapter",
           name="gazebo_controller_adapter",output="screen",
           parameters=[os.path.join(share("hnu_gazebo_adapter"),"config/gazebo_adapter.yaml"),
                       {"use_sim_time":True}]),
      Node(package="hnu_tunnel_controller",executable="navigation_coordinator_node",
           name="navigation_coordinator",output="screen",
           parameters=[os.path.join(share("hnu2027_sentry_bringup"),"config/navigation_coordinator.yaml"),
                       {"use_sim_time":True}]),
      Node(package="hnu2027_sentry_autonomy",executable="simulation_sensors",
           parameters=[{"use_sim_time":True}],output="screen"),
      Node(package="hnu_tunnel_perception",executable="tunnel_detector_node",name="tunnel_detector",
           output="screen",parameters=[{"use_sim_time":True,"tunnel_id":"blue_spawn_tunnel",
             "center_x_m":-2.5,"center_y_m":3.5,"axis_yaw_rad":0.0,"floor_elevation_m":0.1005,
             "expected_width_m":0.8,"roi_length_m":1.4,"roi_width_m":1.2,
             "input_cloud_topic":"/tunnel/cloud","observation_topic":"/tunnel/observation",
             "minimum_samples":40,"minimum_roof_points":12,"minimum_wall_points":6,
             "allow_roof_edge_width":True,"roof_exclusion_m":0.012,
             "accumulation_seconds":30.0,"max_input_age_seconds":0.5}]),
      Node(package="hnu2027_sentry_autonomy",executable="tunnel_mission_node",
           parameters=[{"use_sim_time":True}],output="screen"),
    ]
    for package,executable,name in [
      ("nav2_map_server","map_server","map_server"),("nav2_amcl","amcl","amcl"),
      ("nav2_planner","planner_server","planner_server"),
      ("nav2_controller","controller_server","controller_server"),
      ("nav2_bt_navigator","bt_navigator","bt_navigator")]:
        nodes.append(Node(package=package,executable=executable,name=name,output="screen",
          parameters=[params],remappings=[("cmd_vel","/navigation/cmd_vel")]))
    nodes.append(Node(package="nav2_lifecycle_manager",executable="lifecycle_manager",
      name="navigation_lifecycle_manager",output="screen",
      parameters=[{"use_sim_time":True,"autostart":True,
       "node_names":["map_server","amcl","planner_server","controller_server","bt_navigator"],
       "bond_timeout":8.0}]))
    nodes.append(Node(package="rviz2",executable="rviz2",name="autonomy_rviz",
      arguments=["-d",os.path.join(own,"config/autonomy.rviz")],
      parameters=[{"use_sim_time":True}],output="screen",
      condition=IfCondition(LaunchConfiguration("use_rviz"))))
    return nodes

def generate_launch_description():
    return LaunchDescription([
      DeclareLaunchArgument("gui",default_value="true"),
      DeclareLaunchArgument("use_rviz",default_value="false"),
      DeclareLaunchArgument("block_tunnel",default_value="false"),
      DeclareLaunchArgument("lidar_geometry",default_value="collision"),
      DeclareLaunchArgument("simulation_domain_id",default_value="182"),
      DeclareLaunchArgument("ign_partition",default_value="hnu_autonomy_sim"),
      SetEnvironmentVariable("ROS_DOMAIN_ID",LaunchConfiguration("simulation_domain_id")),
      SetEnvironmentVariable("IGN_PARTITION",LaunchConfiguration("ign_partition")),
      OpaqueFunction(function=start)])
