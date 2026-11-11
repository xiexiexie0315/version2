from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from hnu2027_sentry_autonomy.world_builder import build_world

def workspace():
    return Path(__file__).resolve().parents[3]

def test_real_field_robot_and_lidar_are_composed_without_editing_original(tmp_path):
    r=workspace()
    description=r/"src/hnu2027_sentry_description"
    simulation=r/"src/hnu2027_sentry_simulation"
    source=description/"resource/models/hnu2027_sentry/model.sdf"
    before=source.read_bytes()
    result=ET.parse(build_world(description,simulation,tmp_path/"world.sdf")).getroot()
    assert source.read_bytes()==before
    world=result.find("world")
    assert world.find("model[@name='rmul_2027_3v3']") is not None
    robot=world.find("model[@name='hnu2027_sentry']")
    assert len(robot.findall(".//sensor[@type='gpu_lidar']"))==2
    assert robot.find("joint[@name='fold_roll']").get("type")=="revolute"
    assert world.find("model[@name='navigation_floor']/link/collision") is not None

def test_map_image_resolves_on_disk():
    path=workspace()/"src/hnu2027_sentry_nav/hnu_tunnel_bringup/map/simulation/rmul_2027_3v3.yaml"
    data=yaml.safe_load(path.read_text())
    assert (path.parent/data["image"]).is_file()

def test_both_navigation_trees_do_not_require_unconnected_recovery_motion():
    root=Path(__file__).resolve().parents[1]
    for name in ("navigate.xml","navigate_through.xml"):
        tree=ET.parse(root/"config"/name)
        assert not any(tree.findall(".//"+tag) for tag in ("Spin","BackUp","Wait"))
        assert tree.find(".//FollowPath") is not None
