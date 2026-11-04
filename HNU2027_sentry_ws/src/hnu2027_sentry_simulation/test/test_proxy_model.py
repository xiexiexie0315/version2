from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "resource/models/hnu2027_sentry_proxy/model.sdf"


def test_fold_joint_contract_and_limits():
    root = ET.parse(MODEL).getroot()
    model = root.find("model")
    joint = model.find("joint[@name='fold_joint']")
    assert joint is not None
    assert joint.attrib["type"] == "prismatic"
    assert joint.findtext("parent") == "base_link"
    assert joint.findtext("child") == "upper_link"
    assert float(joint.findtext("axis/limit/lower")) == -0.16
    assert float(joint.findtext("axis/limit/upper")) == 0.0


def test_required_stable_frames_exist():
    root = ET.parse(MODEL).getroot()
    names = {link.attrib["name"] for link in root.findall("model/link")}
    assert {"base_link", "upper_link", "lidar_link", "imu_link"} <= names
