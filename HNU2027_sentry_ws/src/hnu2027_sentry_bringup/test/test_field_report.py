import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "validate_field_report.py"
spec = importlib.util.spec_from_file_location("field_report", SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def report():
    return json.loads((ROOT / "config" / "field_validation_report.example.json").read_text(encoding="utf-8"))


def test_shadow_or_manual_report_schema_is_accepted_without_automatic_approval():
    valid, _ = module.validate(report())
    assert valid


def test_automatic_approval_requires_reviewer_rosbag_and_consistent_trials():
    evidence = report()
    evidence["autonomous_validation_approved"] = True
    valid, _ = module.validate(evidence)
    assert not valid
    evidence["reviewer"] = "safety-reviewer"
    evidence["rosbag_paths"] = ["/data/tunnel_trial_001.db3"]
    evidence["trials"] = {"total": 3, "complete": 2, "safe_failures": 1}
    valid, _ = module.validate(evidence)
    assert valid
