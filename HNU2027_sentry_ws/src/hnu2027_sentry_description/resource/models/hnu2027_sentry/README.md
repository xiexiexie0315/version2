# Original simplified robot, converted from MuJoCo

Source: HUN_sentry_forward4/models/robots/simplified/robot.xml, compiled using
models/scenes/rmul2027_3v3.xml so inherited defaults and materials are resolved.
Mechanical estimates originate in configs/models/simplified.toml.

15 original bodies are preserved as 11 links plus 4 massless armor frames.
Armor geometries are attached to the chassis, preserving their original zero
additional mass. Total mass: 24.815 kg. 137 visible geometries are retained;
96 lidar debug-ray overlays are omitted. Four wheels retain 12 visual rollers
each and the original cylinder contact proxies.

The fold joint is fold_roll, X-axis revolute, 0 to pi/2 radians. It is NOT the
old proxy's vertical slider. big_yaw, small_yaw and pitch retain original axes
and ranges. MID-360 remains attached to upper_platform, outside the fold chain.

Native Gazebo command: /model/hnu2027_sentry/joint/<joint>/cmd_pos
Message: ignition.msgs.Double. Feedback: /model/hnu2027_sentry/joint_state.

Scope: visual/rigid-body migration and native joint debugging. No wheel drive,
ROS FoldState bridge, camera images, lidar point cloud or IMU data is added yet.
Self collision is disabled for this initial inspection model; the source's
pair-specific contact exclusions are not reproduced. This is not a validated
self-collision or automatic-tunnel simulation. Original friction values are
initial settings and do not establish equivalent MuJoCo/Gazebo dynamics.

Regression reference: source_manifest.json contains original compiled body,
geometry and joint properties. test/test_original_model.py checks conversion.
The simulation package's scripts/test_original_model_runtime.py separately
starts an isolated headless world, commands joints, and checks live feedback.
