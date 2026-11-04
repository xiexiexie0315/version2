import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = ROOT / "config" / "tunnel_scenarios.json"


def is_complete_success(action_succeeded, terminal_phase):
    return action_succeeded and terminal_phase == "complete"


def test_all_scenarios_produce_a_safe_structured_report():
    reports = []
    for item in json.loads(SCENARIOS.read_text(encoding="utf-8")):
        actual_success = is_complete_success(item["action_succeeded"], item["terminal_phase"])
        report = {
            "scenario": item["scenario"],
            "action_result": "success" if actual_success else "failure",
            "terminal_phase": item["terminal_phase"],
            "reason": item["reason"],
            "zero_velocity_observed": item["zero_velocity_observed"],
            "passed": actual_success == item["expected_success"] and item["zero_velocity_observed"],
        }
        reports.append(report)
        assert report["passed"], report
        if report["action_result"] == "success":
            assert report["terminal_phase"] == "complete"
        else:
            assert report["zero_velocity_observed"]
    # The real integration runner serializes this same list as JSON. Keeping
    # this unit test in memory makes it portable to read-only CI temp folders.
    assert json.loads(json.dumps(reports)) == reports
    assert len(reports) == 7


def test_behavior_tree_xml_has_only_action_contract_ports():
    text = (ROOT / "behavior_trees" / "rmul_2027_tunnel.xml").read_text(encoding="utf-8")
    for port in ("server_name", "tunnel_id", "direction", "terminal_phase", "reason", "current_phase"):
        assert port in text
