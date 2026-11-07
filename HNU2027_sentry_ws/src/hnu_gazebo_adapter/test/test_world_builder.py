from pathlib import Path
import xml.etree.ElementTree as ET
from hnu_gazebo_adapter.world_builder import build_world

def test_original_model_unchanged_and_real_rotational_fold(tmp_path):
    workspace = Path(__file__).resolve().parents[3]
    description = workspace / "src/hnu2027_sentry_description"
    simulation = workspace / "src/hnu2027_sentry_simulation"
    original = description / "resource/models/hnu2027_sentry/model.sdf"
    before = original.read_bytes()
    output = build_world(description, simulation, tmp_path / "world.sdf")
    assert original.read_bytes() == before
    model = ET.parse(output).getroot().find("world/model[@name='hnu2027_sentry']")
    joint = model.find("joint[@name='fold_roll']")
    assert joint.get("type") == "revolute"
    assert float(joint.findtext("axis/limit/upper")) > 1.57
    assert len(model.findall("joint")) == 10
    assert model.find("plugin[@name='hnu_gazebo_adapter::DriveWatchdog']") is not None
    assert model.find("plugin[@name='ignition::gazebo::systems::OdometryPublisher']") is not None
