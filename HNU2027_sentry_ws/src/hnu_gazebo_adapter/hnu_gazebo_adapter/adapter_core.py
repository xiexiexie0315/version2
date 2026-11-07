"""Simulation interface logic. All completion flags require measured samples."""
import math
from dataclasses import dataclass, field

STOP, SPIN, FOLLOW, DIRECT = range(4)
AUTO, CENTER, STOW, HOLD = range(4)
KEEP, FOLD, UNFOLD = range(3)
JOINTS = ("big_yaw", "small_yaw", "pitch", "fold_roll")

@dataclass
class Config:
    command_timeout: float = 0.3
    feedback_timeout: float = 0.3
    settle_time: float = 0.2
    position_tolerance: float = 0.03
    joint_speed_tolerance: float = 0.08
    stopped_linear: float = 0.035
    stopped_angular: float = 0.04
    spin_speed: float = 0.5
    linear_limit: float = 0.5
    angular_limit: float = 0.8
    linear_acceleration: float = 0.5
    angular_acceleration: float = 0.7
    follow_kp: float = 1.5
    action_timeout: float = 12.0
    folded_position: float = math.pi / 2

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError("invalid config: " + name)
        if self.folded_position > math.pi / 2 + 1e-6:
            raise ValueError("folded position exceeds original model range")

@dataclass
class Command:
    request_id: int = 0
    chassis_mode: int = STOP
    gimbal_mode: int = HOLD
    posture: int = KEEP
    velocity: tuple = (0.0, 0.0, 0.0)
    valid: bool = False
    motion_allowed: bool = False

@dataclass
class Result:
    request_id: int = 0
    chassis_mode: int = STOP
    gimbal_mode: int = HOLD
    valid: bool = False
    stopped: bool = False
    centered: bool = False
    folded: bool = False
    unfolded: bool = False
    gimbal_ready: bool = False
    fault: bool = False
    actual_velocity: tuple = (0.0, 0.0, 0.0)
    drive: tuple = (0.0, 0.0, 0.0)
    targets: dict = field(default_factory=dict)
    execution: str = "idle"
    reason: str = "waiting_measurements"

def finite(values):
    return all(math.isfinite(v) for v in values)

def clamp(v, limit):
    return max(-limit, min(limit, v))

def approach(current, target, maximum_step):
    return current + clamp(target - current, maximum_step)

class GazeboController:
    def __init__(self, config=None):
        self.config = config or Config()
        self.command = Command()
        self.command_at = -math.inf
        self.joints_at = self.odom_at = -math.inf
        self.joint_stamp = self.odom_stamp = -math.inf
        self.positions = {}
        self.joint_speeds = {}
        self.actual_velocity = (0.0, 0.0, 0.0)
        self.targets = {}
        self.drive = (0.0, 0.0, 0.0)
        self.mode, self.gimbal_mode, self.accepted_id = STOP, HOLD, 0
        self.last_tick = None
        self.active_before = False
        self.stable_since = {}
        self.flags = {}
        self.fault = ""
        self.action = None
        self.action_started = 0.0
        self.gimbal_started = None
        self.auto_target = None
        self.auto_target_at = -math.inf

    def settled(self, name, condition, stamp):
        if not condition:
            self.stable_since.pop(name, None)
            self.flags[name] = False
        else:
            since = self.stable_since.setdefault(name, stamp)
            self.flags[name] = stamp - since >= self.config.settle_time

    def receive_joints(self, positions, stamp, now):
        if (set(JOINTS) - set(positions) or not finite(positions.values())
                or not finite((stamp, now))):
            self.joints_at = -math.inf
            return False
        if stamp <= self.joint_stamp:
            if stamp < self.joint_stamp:
                self.fault = "simulation_time_reset"
            return False
        dt = stamp - self.joint_stamp
        self.joint_speeds = {j: (positions[j] - self.positions[j]) / dt
                             for j in JOINTS} if self.positions else {}
        self.positions = {j: positions[j] for j in JOINTS}
        self.joint_stamp, self.joints_at = stamp, now
        if not self.targets:
            self.targets = dict(self.positions)
        still = bool(self.joint_speeds) and all(
            abs(self.joint_speeds[j]) <= self.config.joint_speed_tolerance for j in JOINTS[:3])
        centered = all(abs(self.positions[j]) <= self.config.position_tolerance
                       for j in JOINTS[:3])
        self.settled("centered", centered and still, stamp)
        for name, target in (("folded", self.config.folded_position), ("unfolded", 0.0)):
            self.settled(name, bool(self.joint_speeds) and
                         abs(self.joint_speeds["fold_roll"]) <= self.config.joint_speed_tolerance and
                         abs(self.positions["fold_roll"] - target)
                         <= self.config.position_tolerance, stamp)
        return True

    def receive_odometry(self, velocity, stamp, now):
        if len(velocity) != 3 or not finite((*velocity, stamp, now)):
            self.odom_at = -math.inf
            return False
        if stamp <= self.odom_stamp:
            if stamp < self.odom_stamp:
                self.fault = "simulation_time_reset"
            return False
        self.actual_velocity = tuple(velocity)
        self.odom_stamp, self.odom_at = stamp, now
        self.settled("stopped",
                     math.hypot(*velocity[:2]) <= self.config.stopped_linear and
                     abs(velocity[2]) <= self.config.stopped_angular, stamp)
        return True

    def receive_command(self, command, now):
        valid_modes = (command.chassis_mode in range(4) and
                       command.gimbal_mode in range(4) and command.posture in range(3))
        if not valid_modes or len(command.velocity) != 3 or not finite((*command.velocity, now)):
            self.command = Command()
            self.command_at = -math.inf
            return False
        self.command, self.command_at = command, now
        return True

    def receive_auto_target(self, positions, now):
        # Simulation-only joint targets, not the real MPC world-angle interface.
        if set(positions) != set(JOINTS[:3]) or not finite(positions.values()):
            return False
        if abs(positions["small_yaw"]) > 1.3963 or not -0.349 <= positions["pitch"] <= 0.523:
            return False
        self.auto_target = dict(positions)
        self.auto_target_at = now
        return True

    def tick(self, now):
        c = self.config
        dt = min(0.1, max(0.0, now - self.last_tick)) if self.last_tick is not None else 0.02
        self.last_tick = now
        measured = (bool(self.positions) and
                    0 <= now - self.joints_at <= c.feedback_timeout and
                    0 <= now - self.odom_at <= c.feedback_timeout)
        flags = self.flags if measured else {}
        result = Result(valid=measured, fault=bool(self.fault),
                        actual_velocity=self.actual_velocity,
                        stopped=flags.get("stopped", False),
                        centered=flags.get("centered", False),
                        folded=flags.get("folded", False),
                        unfolded=flags.get("unfolded", False))
        result.gimbal_ready = result.unfolded and not result.fault
        cmd = self.command
        active = (measured and not self.fault and cmd.valid and
                  0 <= now - self.command_at <= c.command_timeout)
        target_drive = (0.0, 0.0, 0.0)
        if not active:
            if self.active_before and self.positions:
                self.targets = dict(self.positions)
            self.mode, self.gimbal_mode = STOP, HOLD
            self.action = None
            self.gimbal_started = None
            result.reason = (self.fault or ("command_inactive" if measured
                                           else "measurements_stale"))
            result.execution = "fault" if self.fault else "holding"
        else:
            self.mode, self.gimbal_mode = cmd.chassis_mode, cmd.gimbal_mode
            self.accepted_id = cmd.request_id
            result.reason = "request_accepted"
            result.execution = "holding"
            if cmd.gimbal_mode in (CENTER, STOW):
                if result.stopped or result.folded:
                    for joint in JOINTS[:3]:
                        self.targets[joint] = 0.0
                    if result.centered:
                        self.gimbal_started = None
                    else:
                        if self.gimbal_started is None:
                            self.gimbal_started = now
                        result.execution = "centering"
                        if now - self.gimbal_started > c.action_timeout:
                            self.fault = "centering_timeout"
                else:
                    result.reason = "waiting_stopped_before_center"
            else:
                self.gimbal_started = None
                if (cmd.gimbal_mode == AUTO and result.unfolded and self.auto_target
                        and now - self.auto_target_at <= c.command_timeout):
                    self.targets.update(self.auto_target)
            requested = None
            if cmd.posture == FOLD:
                if result.stopped and result.centered:
                    requested = c.folded_position
                else:
                    result.reason = "waiting_stopped_and_centered_before_fold"
            elif cmd.posture == UNFOLD:
                if result.stopped:
                    requested = 0.0
                else:
                    result.reason = "waiting_stopped_before_unfold"
            if requested is not None:
                if self.action is None or self.action != requested:
                    self.action = requested
                    self.action_started = now
                self.targets["fold_roll"] = requested
            if self.action is not None:
                completed = result.folded if self.action else result.unfolded
                if completed:
                    result.execution = "idle"
                else:
                    result.execution = "folding" if self.action else "unfolding"
                    if now - self.action_started > c.action_timeout:
                        self.fault = "fold_action_timeout"
            if cmd.motion_allowed and cmd.chassis_mode != STOP:
                # Fold motion and chassis motion cannot be requested together.
                moving_fold = abs(self.targets.get("fold_roll", 0.0) -
                                  self.positions.get("fold_roll", 0.0)) > c.position_tolerance
                if not moving_fold:
                    vx, vy, wz = cmd.velocity
                    if cmd.chassis_mode == SPIN:
                        if result.unfolded:
                            wz = c.spin_speed
                        else:
                            vx = vy = wz = 0.0
                    elif cmd.chassis_mode == FOLLOW:
                        if result.unfolded:
                            angle = self.positions["big_yaw"] + self.positions["small_yaw"]
                            wz = c.follow_kp * math.atan2(math.sin(angle), math.cos(angle))
                        else:
                            vx = vy = wz = 0.0
                    target_drive = (clamp(vx, c.linear_limit), clamp(vy, c.linear_limit),
                                    clamp(wz, c.angular_limit))
        self.active_before = active
        if self.fault or not measured:
            # No stale sensor-based continuation. Hold last observed joint position.
            target_drive = (0.0, 0.0, 0.0)
            if self.positions:
                self.targets = dict(self.positions)
            self.mode, self.gimbal_mode = STOP, HOLD
            result.fault = bool(self.fault)
            result.gimbal_ready = False
            result.execution = "fault" if self.fault else "holding"
            result.reason = self.fault or "measurements_stale"
        self.drive = tuple(approach(v, t, (c.angular_acceleration if i == 2
                           else c.linear_acceleration) * dt)
                           for i, (v, t) in enumerate(zip(self.drive, target_drive)))
        result.request_id = self.accepted_id
        result.chassis_mode, result.gimbal_mode = self.mode, self.gimbal_mode
        result.drive, result.targets = self.drive, dict(self.targets)
        return result
