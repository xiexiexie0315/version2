"""Stage-one HNU 2027 entry point.

This launch file intentionally starts no motion, simulator, perception or
folding nodes. It reserves a stable top-level entry point while the individual
subsystems are introduced and verified in later stages.
"""

from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="HNU 2027 stage-one scaffold: no robot-control nodes started."),
    ])
