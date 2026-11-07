"""Process-level ROS test in an isolated domain; never connects a transport."""
import os
import subprocess
import time

def test_coordinator_topics():
    os.environ["ROS_DOMAIN_ID"] = str(150 + os.getpid() % 30)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import Twist, TwistStamped
    from hnu_sentry_interfaces.msg import TraverseControl, MotionControlCommand, LowerControllerState

    rclpy.init()
    node = rclpy.create_node("coordinator_test_driver")
    prefix = "/stage1_test_" + str(os.getpid())
    mappings = {
        "/navigation/cmd_vel": prefix + "/nav",
        "/tunnel/cmd_vel": prefix + "/tunnel",
        "/tunnel/control_request": prefix + "/request",
        "/robot/controller_state": prefix + "/state",
        "/robot/control_command": prefix + "/command",
        "/robot/emergency_stop": prefix + "/stop",
    }
    nav = node.create_publisher(Twist, mappings["/navigation/cmd_vel"], 1)
    tunnel = node.create_publisher(TwistStamped, mappings["/tunnel/cmd_vel"], 1)
    request = node.create_publisher(TraverseControl, mappings["/tunnel/control_request"], 1)
    state = node.create_publisher(LowerControllerState, mappings["/robot/controller_state"], 1)
    received = []
    subscription = node.create_subscription(
        MotionControlCommand, mappings["/robot/control_command"],
        lambda msg: received.append(msg), 10)
    exe = os.path.join(get_package_prefix("hnu_tunnel_controller"),
                       "lib/hnu_tunnel_controller/navigation_coordinator_node")
    args = [exe, "--ros-args"]
    for source, target in mappings.items():
        args += ["-r", source + ":=" + target]
    process = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    req = TraverseControl()
    req.request_id = 42
    status = LowerControllerState()
    status.accepted_request_id = 42
    status.valid = status.stopped = status.centered = True
    status.unfolded = status.gimbal_ready = True
    status.chassis_mode = MotionControlCommand.CHASSIS_SPIN
    n = Twist()
    n.linear.x, n.linear.y, n.angular.z = 0.5, 0.2, 0.8
    t = TwistStamped()
    t.header.frame_id = "base_link"
    t.twist.linear.x, t.twist.linear.y, t.twist.angular.z = 0.1, -0.03, -0.2

    def wait_for(predicate, send_request=True, send_state=True, send_tunnel=True):
        received.clear()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            assert process.poll() is None, process.stderr.read().decode()
            stamp = node.get_clock().now().to_msg()
            req.header.stamp = status.header.stamp = t.header.stamp = stamp
            nav.publish(n)
            if send_request:
                request.publish(req)
            if send_state:
                state.publish(status)
            if send_tunnel:
                tunnel.publish(t)
            rclpy.spin_once(node, timeout_sec=0.03)
            for msg in received:
                if predicate(msg):
                    return msg
            received.clear()
            time.sleep(0.01)
        raise AssertionError("Expected coordinator output was not observed")

    try:
        msg = wait_for(lambda m: m.motion_allowed and m.source == m.SOURCE_NAV2)
        assert msg.chassis_mode == msg.CHASSIS_SPIN
        assert abs(msg.velocity.linear.y - 0.2) < 1e-6
        assert msg.velocity.angular.z == 0.0

        req.phase = req.ALIGN
        status.centered = False
        wait_for(lambda m: not m.motion_allowed and m.reason == "waiting_centered")
        status.centered = True
        wait_for(lambda m: m.reason == "waiting_chassis_mode_ack")
        status.chassis_mode = MotionControlCommand.CHASSIS_NAV_DIRECT
        msg = wait_for(lambda m: m.motion_allowed and m.source == m.SOURCE_TUNNEL)
        assert abs(msg.velocity.linear.y + 0.03) < 1e-6

        req.phase = req.FOLD
        msg = wait_for(lambda m: m.posture_request == m.POSTURE_FOLD)
        assert not msg.motion_allowed and not msg.allow_fire
        req.phase = req.TRAVERSE
        wait_for(lambda m: m.reason == "fold_not_verified")
        status.folded, status.unfolded = True, False
        wait_for(lambda m: m.motion_allowed and m.source == m.SOURCE_TUNNEL)
        msg = wait_for(lambda m: m.reason == "tunnel_command_timeout",
                       send_tunnel=False)
        assert not msg.motion_allowed and msg.source == msg.SOURCE_NONE
        wait_for(lambda m: m.reason == "control_request_timeout",
                 send_request=False)
        wait_for(lambda m: m.reason == "controller_feedback_timeout",
                 send_state=False)
        # Restored inputs cannot silently release tunnel ownership.
        req.phase = req.NORMAL
        status.folded, status.unfolded = False, True
        wait_for(lambda m: m.reason == "explicit_complete_required")
        req.phase = req.UNFOLD
        wait_for(lambda m: m.posture_request == m.POSTURE_UNFOLD)
        req.phase = req.COMPLETE
        wait_for(lambda m: m.reason == "waiting_chassis_mode_ack")
        status.chassis_mode = MotionControlCommand.CHASSIS_SPIN
        wait_for(lambda m: m.motion_allowed and m.source == m.SOURCE_NAV2)
    finally:
        process.terminate()
        try:
            process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate()
        node.destroy_node()
        rclpy.shutdown()
