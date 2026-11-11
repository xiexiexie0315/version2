"""Compose the actual field and original robot with simulated lidar sensors."""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET
from hnu_gazebo_adapter.world_builder import build_world as inspection_world

def build_world(description, simulation, destination, block_tunnel=False, lidar_collision_geometry=True):
    inspection_world(description, simulation, destination)
    robot = ET.parse(destination).getroot().find("world/model[@name='hnu2027_sentry']")
    tree = ET.parse(Path(simulation) / "resource/worlds/rmul_2027_3v3_world.sdf")
    tree.getroot().set("version","1.9")
    world = tree.getroot().find("world")
    world.set("name", "autonomy_sim")
    # The inspection roof is translucent; use opaque physical surfaces for GPU lidar.
    field=copy.deepcopy(ET.parse(Path(simulation)/"resource/models/rmul_2027_3v3/model.sdf").getroot().find("model"))
    for include in list(world.findall("include")): world.remove(include)
    for name in ("blue_spawn_tunnel_roof","red_spawn_tunnel_roof"):
        material=field.find("link[@name='"+name+"']/visual/material/diffuse")
        rgba=material.text.split();rgba[-1]="1";material.text=" ".join(rgba)
    world.append(field)
    robot.find("pose").text = "-4 3.5 0.1055 0 0 0"
    odom = robot.find("plugin[@name='ignition::gazebo::systems::OdometryPublisher']")
    odom.find("odom_frame").text = "odom"
    ET.SubElement(odom, "xyz_offset").text = "0 0 0.143"
    # CAD collision mesh contains perimeter/obstacles but no floor at these poses.
    ground = ET.SubElement(world, "model", {"name": "navigation_floor"})
    ET.SubElement(ground, "static").text = "true"
    link = ET.SubElement(ground, "link", {"name": "floor"})
    collision = ET.SubElement(link, "collision", {"name": "floor_collision"})
    ET.SubElement(collision, "pose").text = "0 0 0.0505 0 0 0"
    geom = ET.SubElement(collision, "geometry")
    ET.SubElement(ET.SubElement(geom, "box"), "size").text = "12.56 8.56 0.1"
    sensors = ET.SubElement(world, "plugin", {
        "filename": "ignition-gazebo-sensors-system",
        "name": "ignition::gazebo::systems::Sensors"})
    ET.SubElement(sensors, "render_engine").text = "ogre2"
    # A dedicated chassis-fixed navigation scan avoids cloud slicing at a changing fold pose.
    navlink = ET.SubElement(robot, "link", {"name": "nav_lidar_link"})
    ET.SubElement(navlink, "pose", {"relative_to": "chassis"}).text = "0 0 0.04 0 0 0"
    joint = ET.SubElement(robot, "joint", {"name": "nav_lidar_fixed", "type": "fixed"})
    ET.SubElement(joint, "parent").text = "chassis"
    ET.SubElement(joint, "child").text = "nav_lidar_link"
    def lidar(parent, name, topic, vertical):
        sensor = ET.SubElement(parent, "sensor", {"name": name, "type": "gpu_lidar"})
        ET.SubElement(sensor, "topic").text = topic
        ET.SubElement(sensor, "update_rate").text = "12"
        ET.SubElement(sensor, "always_on").text = "true"
        spec = ET.SubElement(sensor, "lidar")
        scan = ET.SubElement(spec, "scan")
        horizontal = ET.SubElement(scan, "horizontal")
        for key, value in {"samples": "480" if vertical else "720", "resolution": "1",
                           "min_angle": "-3.14159265", "max_angle": "3.14159265"}.items():
            ET.SubElement(horizontal, key).text = value
        if vertical:
            v = ET.SubElement(scan, "vertical")
            for key,value in {"samples":"96","resolution":"1",
                              "min_angle":"-0.6","max_angle":"0.6"}.items():
                ET.SubElement(v,key).text=value
        rg = ET.SubElement(spec,"range")
        for key,value in {"min":"0.08","max":"10","resolution":"0.005"}.items():
            ET.SubElement(rg,key).text=value
    lidar(navlink, "navigation_lidar", "/simulation/raw_scan", False)
    lidar(robot.find("link[@name='mid360']"), "tunnel_lidar", "/simulation/raw_tunnel_scan", True)
    if block_tunnel:
        obstacle=ET.SubElement(world,"model",{"name":"tunnel_test_obstacle"})
        ET.SubElement(obstacle,"static").text="true"
        ET.SubElement(obstacle,"pose").text="-2.5 3.5 0.1905 0 0 0"
        link=ET.SubElement(obstacle,"link",{"name":"block"})
        for kind in ("visual","collision"):
            item=ET.SubElement(link,kind,{"name":kind})
            geometry=ET.SubElement(item,"geometry")
            ET.SubElement(ET.SubElement(geometry,"box"),"size").text="0.12 0.60 0.18"
            if kind=="visual":
                material=ET.SubElement(item,"material")
                ET.SubElement(material,"diffuse").text="0.9 0.1 0.1 1"
    world.append(robot)
    if lidar_collision_geometry:
        # GPU rays normally render visual meshes, unlike collision-based CPU rays.
        # Derive sensor surfaces from existing collisions without changing physics.
        for link in world.findall(".//link"):
            for visual in link.findall("visual"):
                flag=visual.find("visibility_flags")
                if flag is None:flag=ET.SubElement(visual,"visibility_flags")
                flag.text="1"
            for collision in link.findall("collision"):
                visual=ET.SubElement(link,"visual",{"name":"lidar_"+collision.get("name")})
                pose=collision.find("pose")
                if pose is not None:visual.append(copy.deepcopy(pose))
                visual.append(copy.deepcopy(collision.find("geometry")))
                ET.SubElement(visual,"visibility_flags").text="2"
                material=ET.SubElement(visual,"material")
                ET.SubElement(material,"diffuse").text="0.35 0.40 0.45 1"
        for spec in world.findall(".//sensor/lidar"):
            ET.SubElement(spec,"visibility_mask").text="2"
    ET.indent(tree, space="  ")
    tree.write(destination,encoding="utf-8",xml_declaration=True)
    return str(destination)
