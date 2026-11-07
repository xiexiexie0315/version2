"""ROS facade for the measured-feedback simulation adapter."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from hnu_sentry_interfaces.msg import MotionControlCommand, LowerControllerState
from .adapter_core import GazeboController, Command, Config, JOINTS

class GazeboAdapterNode(Node):
    def __init__(self):
        super().__init__("gazebo_controller_adapter")
        config = Config(**{k: self.declare_parameter(k, v).value
                           for k, v in vars(Config()).items()})
        self.core = GazeboController(config)
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.model = self.declare_parameter("model_name", "hnu2027_sentry").value
        root = "/model/" + self.model
        self.create_subscription(MotionControlCommand, "/robot/control_command",
                                 self.command, 1)
        self.create_subscription(JointState, root + "/joint_state", self.joints,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, root + "/odometry", self.odometry,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, "/simulation/gimbal_target",
                                 self.gimbal_target, 1)
        self.drive_pub = self.create_publisher(Twist, root + "/cmd_vel", 1)
        self.joint_pubs = {j: self.create_publisher(
            Float64, root + "/joint/" + j + "/cmd_pos", 1) for j in JOINTS}
        self.state_pub = self.create_publisher(LowerControllerState,
                                               "/robot/controller_state", 1)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.02, self.update, clock=self.wall_clock)

    def stamp(self, header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def command(self, msg):
        age = self.get_clock().now().nanoseconds * 1e-9 - self.stamp(msg.header)
        now = time.monotonic()
        valid = (msg.command_valid and msg.header.frame_id == self.base_frame and
                 math.isfinite(age) and 0 <= age <= self.core.config.command_timeout)
        c = Command(msg.request_id, msg.chassis_mode, msg.gimbal_mode,
                    msg.posture_request,
                    (msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.angular.z),
                    valid, msg.motion_allowed)
        self.core.receive_command(c, now - max(0.0, age) if valid else now)

    def joints(self, msg):
        if len(msg.position) != len(msg.name):
            self.core.joints_at = -math.inf
            return
        self.core.receive_joints(dict(zip(msg.name, msg.position)),
                                 self.stamp(msg.header), time.monotonic())

    def odometry(self, msg):
        v = msg.twist.twist
        self.core.receive_odometry((v.linear.x, v.linear.y, v.angular.z),
                                   self.stamp(msg.header), time.monotonic())

    def gimbal_target(self, msg):
        age = self.get_clock().now().nanoseconds * 1e-9 - self.stamp(msg.header)
        if (len(msg.name) != len(msg.position) or msg.header.frame_id != self.base_frame or
                not 0 <= age <= self.core.config.command_timeout):
            return
        self.core.receive_auto_target(dict(zip(msg.name, msg.position)),
                                       time.monotonic() - age)

    def update(self):
        out = self.core.tick(time.monotonic())
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = out.drive
        self.drive_pub.publish(twist)
        for name, target in out.targets.items():
            self.joint_pubs[name].publish(Float64(data=target))
        state = LowerControllerState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = self.base_frame
        state.accepted_request_id = out.request_id
        state.chassis_mode, state.gimbal_mode = out.chassis_mode, out.gimbal_mode
        for name in ("valid", "stopped", "centered", "folded", "unfolded",
                     "gimbal_ready", "fault"):
            setattr(state, name, getattr(out, name))
        state.velocity.linear.x, state.velocity.linear.y, state.velocity.angular.z = out.actual_velocity
        state.execution_state = {
            "idle": state.EXEC_IDLE, "holding": state.EXEC_HOLDING,
            "centering": state.EXEC_CENTERING, "folding": state.EXEC_FOLDING,
            "unfolding": state.EXEC_UNFOLDING, "fault": state.EXEC_FAULT}[out.execution]
        state.reason = out.reason
        self.state_pub.publish(state)

def main():
    rclpy.init()
    node = GazeboAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.drive_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
