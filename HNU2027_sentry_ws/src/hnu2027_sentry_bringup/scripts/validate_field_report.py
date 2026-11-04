#!/usr/bin/env python3
"""Validate field evidence before changing autonomous deployment approval."""

import json
import sys
from pathlib import Path

REQUIRED = {
    "field_geometry", "robot_envelope", "fold_execution", "braking",
    "lidar_extrinsic", "trials", "autonomous_validation_approved", "reviewer", "rosbag_paths",
}


def validate(report):
    missing = REQUIRED - set(report)
    if missing:
        return False, "missing:" + ",".join(sorted(missing))
    if report["autonomous_validation_approved"]:
        if not report["reviewer"]:
            return False, "autonomous approval requires reviewer"
        if not report["rosbag_paths"]:
            return False, "autonomous approval requires rosbag evidence"
        trials = report["trials"]
        if trials["total"] <= 0 or trials["complete"] + trials["safe_failures"] != trials["total"]:
            return False, "trial accounting invalid"
    return True, "report_schema_valid"


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: validate_field_report.py REPORT.json")
    ok, reason = validate(json.loads(Path(sys.argv[1]).read_text(encoding="utf-8")))
    print(json.dumps({"valid": ok, "reason": reason}))
    raise SystemExit(0 if ok else 1)
