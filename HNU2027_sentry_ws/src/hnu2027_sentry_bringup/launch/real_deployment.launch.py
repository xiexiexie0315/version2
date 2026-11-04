"""Fail-closed real-robot deployment entry point for tunnel validation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _validate(context):
    mode = LaunchConfiguration("deployment_mode").perform(context)
    enable_control = LaunchConfiguration("enable_control").perform(context).lower() == "true"
    validation = LaunchConfiguration("autonomous_validation_approved").perform(context).lower() == "true"
    if mode not in ("shadow", "manual_permit", "autonomous"):
        raise RuntimeError("deployment_mode must be shadow, manual_permit, or autonomous")
    if mode == "shadow" and enable_control:
        raise RuntimeError("shadow mode must not enable control")
    if mode == "autonomous" and (not enable_control or not validation):
        raise RuntimeError("autonomous mode requires enable_control=true and approved field validation")
    return [LogInfo(msg=f"Tunnel real deployment mode={mode}; control_enabled={enable_control}.")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("deployment_mode", default_value="shadow"),
        DeclareLaunchArgument("enable_control", default_value="false"),
        DeclareLaunchArgument("autonomous_validation_approved", default_value="false"),
        OpaqueFunction(function=_validate),
        LogInfo(msg="This launch validates deployment permissions only; it does not start control nodes by itself."),
    ])
