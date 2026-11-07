"""Construct a separate control world from the original model, without editing it."""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

def build_world(description_share, simulation_share, destination):
    model = copy.deepcopy(ET.parse(Path(description_share) /
        "resource/models/hnu2027_sentry/model.sdf").getroot().find("model"))
    world_tree = ET.parse(Path(simulation_share) /
        "resource/worlds/original_model_debug.sdf")
    world = world_tree.getroot().find("world")
    world.set("name", "coordinator_sim")
    for include in list(world.findall("include")):
        world.remove(include)
    pose = model.find("pose")
    if pose is None:
        pose = ET.SubElement(model, "pose")
    pose.text = "0 0 0.01 0 0 0"
    plugin = ET.SubElement(model, "plugin", {
        "filename": "ignition-gazebo-velocity-control-system",
        "name": "ignition::gazebo::systems::VelocityControl"})
    ET.SubElement(plugin, "topic").text = "/model/hnu2027_sentry/cmd_vel"
    plugin = ET.SubElement(model, "plugin", {
        "filename": "ignition-gazebo-odometry-publisher-system",
        "name": "ignition::gazebo::systems::OdometryPublisher"})
    for name, value in {"odom_frame": "sim_world", "robot_base_frame": "base_link",
                        "dimensions": "3", "odom_publish_frequency": "50",
                        "odom_topic": "/model/hnu2027_sentry/odometry"}.items():
        ET.SubElement(plugin, name).text = value
    plugin = ET.SubElement(model, "plugin", {
        "filename": "libhnu_drive_watchdog.so",
        "name": "hnu_gazebo_adapter::DriveWatchdog"})
    ET.SubElement(plugin, "topic").text = "/model/hnu2027_sentry/cmd_vel"
    world.append(model)
    # This is an open inspection area, not a certified tunnel-clearance world.
    ET.indent(world_tree, space="  ")
    world_tree.write(destination, encoding="utf-8", xml_declaration=True)
    return str(destination)
