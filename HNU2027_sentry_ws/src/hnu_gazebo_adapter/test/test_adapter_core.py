import math
from hnu_gazebo_adapter.adapter_core import (
    GazeboController, Config, Command, JOINTS, STOP, SPIN, DIRECT,
    AUTO, CENTER, STOW, KEEP, FOLD, UNFOLD)

def sample(core, now, fold=0.0, gimbal=(0.0, 0.0, 0.0), velocity=(0.0, 0.0, 0.0)):
    p = dict(zip(JOINTS, (*gimbal, fold)))
    core.receive_joints(p, now, now)
    core.receive_odometry(velocity, now, now)
    return core.tick(now)

def settle(core, start=1.0, **kwargs):
    out = None
    for i in range(8):
        out = sample(core, start + i * 0.05, **kwargs)
    return out

def test_no_completion_without_actual_feedback():
    core = GazeboController()
    core.receive_command(Command(1, DIRECT, STOW, FOLD, valid=True), 1.0)
    out = core.tick(1.1)
    assert not out.valid and not out.folded and not out.centered
    out = settle(core)
    assert out.unfolded and out.stopped and out.centered
    core.receive_command(Command(1, DIRECT, STOW, FOLD, valid=True), 1.4)
    out = sample(core, 1.4)
    assert out.targets["fold_roll"] == math.pi / 2
    assert not out.folded
    # Time passing and accepted commands cannot substitute for measured movement.
    for i in range(10):
        t = 1.5 + i * 0.05
        core.receive_command(Command(1, DIRECT, STOW, FOLD, valid=True), t)
        out = sample(core, t)
        assert not out.folded
    out = settle(core, 2.1, fold=math.pi / 2)
    assert out.folded and not out.unfolded

def test_stopped_uses_actual_velocity_not_zero_request():
    core = GazeboController()
    out = settle(core, velocity=(0.0, 0.0, 0.5))
    assert not out.stopped
    core.receive_command(Command(2, DIRECT, CENTER, valid=True), 1.4)
    out = sample(core, 1.4, velocity=(0.0, 0.0, 0.4))
    assert not out.stopped
    out = settle(core, 1.5)
    assert out.stopped

def test_fold_interlock_requires_center_and_stop():
    core = GazeboController()
    settle(core, gimbal=(0.3, 0.2, 0.1), velocity=(0.0, 0.0, 0.4))
    core.receive_command(Command(3, DIRECT, STOW, FOLD, valid=True), 1.4)
    out = sample(core, 1.4, gimbal=(0.3, 0.2, 0.1), velocity=(0.0, 0.0, 0.4))
    assert out.targets["fold_roll"] == 0.0
    assert out.reason == "waiting_stopped_and_centered_before_fold"

def test_command_timeout_stops_spin_but_retains_actual_pose():
    core = GazeboController()
    settle(core)
    for i in range(20):
        t = 1.4 + i * 0.05
        core.receive_command(Command(4, SPIN, AUTO, valid=True, motion_allowed=True), t)
        out = sample(core, t)
    assert out.drive[2] > 0.4
    for i in range(30):
        out = sample(core, 2.4 + i * 0.05)
    assert out.drive == (0.0, 0.0, 0.0)
    assert out.chassis_mode == STOP

def test_duplicate_measurements_cannot_keep_feedback_alive():
    core = GazeboController()
    settle(core)
    p = dict(zip(JOINTS, (0.0, 0.0, 0.0, 0.0)))
    assert not core.receive_joints(p, 1.35, 2.0)
    assert not core.receive_odometry((0.0, 0.0, 0.0), 1.35, 2.0)
    assert not core.tick(2.0).valid

def test_repeated_fold_request_does_not_restart_timeout():
    core = GazeboController(Config(action_timeout=0.5))
    settle(core)
    for i in range(20):
        t = 1.4 + i * 0.05
        core.receive_command(Command(5, DIRECT, STOW, FOLD, valid=True), t)
        out = sample(core, t)
    assert out.fault and not out.folded
    assert out.reason == "fold_action_timeout"

def test_unfold_uses_original_rotational_joint():
    core = GazeboController()
    settle(core, fold=math.pi / 2)
    core.receive_command(Command(6, DIRECT, STOW, UNFOLD, valid=True), 1.4)
    out = sample(core, 1.4, fold=math.pi / 2)
    assert out.targets["fold_roll"] == 0.0
    assert not out.unfolded

def test_motion_authorization_false_never_spins():
    core = GazeboController()
    settle(core)
    core.receive_command(Command(7, SPIN, AUTO, valid=True, motion_allowed=False), 1.4)
    out = sample(core, 1.4)
    assert out.request_id == 7 and out.chassis_mode == SPIN
    assert out.drive == (0.0, 0.0, 0.0)

def test_invalid_velocity_is_rejected():
    core = GazeboController()
    settle(core)
    assert not core.receive_command(
        Command(8, DIRECT, AUTO, velocity=(math.nan, 0.0, 0.0), valid=True), 1.4)
    assert sample(core, 1.4).drive == (0.0, 0.0, 0.0)

def test_simulation_reset_requires_restart():
    core = GazeboController()
    settle(core)
    sample(core, 0.0)
    assert core.tick(1.4).fault


def test_fold_completion_is_independent_of_normal_gimbal_sweep():
    core = GazeboController()
    settle(core)
    for i in range(8):
        out = sample(core, 1.4 + i * 0.05, gimbal=(i * 0.05, 0.2, 0.1))
    assert out.unfolded and not out.centered
