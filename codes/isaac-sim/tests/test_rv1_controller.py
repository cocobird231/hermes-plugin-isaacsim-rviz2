"""Physics-independent control and tensor-frame regression checks."""
import math
from pathlib import Path
import sys
import subprocess
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rv1.controller import RV1Controller
from robots.rv1_tune_control import RV1Controller as CanonicalController, _Ros2Session, start_ros2


class SessionEdit:
    def __init__(self, stage, layer):
        assert layer == "session"
        self.stage = stage

    def __enter__(self):
        self.previous = self.stage.edit_layer
        self.stage.edit_layer = "session"

    def __exit__(self, *args):
        self.stage.edit_layer = self.previous


class Attribute:
    def __init__(self, stage):
        self.stage, self.value, self.writes = stage, 0.0, []

    def Set(self, value):
        assert self.stage.edit_layer == "session", "Control must not author the asset/root layer"
        self.value = value
        self.writes.append(value)


class Tensor:
    def __init__(self, row):
        self.row = row

    def numpy(self):
        return [self.row]


class BodyView:
    count = 1

    def __init__(self):
        self.pose = [3.0, 4.0, 5.0, 0.0, 0.0, 0.0, 1.0]
        self.velocity = [0.0] * 6

    def get_transforms(self):
        return Tensor(self.pose)

    def get_velocities(self):
        return Tensor(self.velocity)


class SimulationView:
    def __init__(self):
        self.body, self.bindings = BodyView(), []

    def create_rigid_body_view(self, path):
        self.bindings.append(path)
        return self.body


@pytest.fixture
def controller():
    # Exercise pure control + session-layer writes without Isaac dependencies.
    item = object.__new__(RV1Controller)
    item._stage = SimpleNamespace(edit_layer="root", GetSessionLayer=lambda: "session")
    item._Usd = SimpleNamespace(EditContext=SessionEdit)
    item._geometry = {"R": 0.35, "L": 2.2, "W": 2.08}
    item._chassis_path = "/World/rv1/base_link/chassis_link"
    item._wheels = {name: {"pos": (x, y), "steer_target": Attribute(item.stage), "drive_target": Attribute(item.stage)}
                    for name, x, y in (("fl", 1.1, 1.04), ("fr", 1.1, -1.04),
                                       ("rl", -1.1, 1.04), ("rr", -1.1, -1.04))}
    item._steer_now = {name: 0.0 for name in item._wheels}
    item._drive_now = dict(item._steer_now)
    item._flipped = {name: False for name in item._wheels}
    item._command = [0.0, 0.0, 0.0]
    item._tensor_sim = item._tensor_body = None
    view = SimulationView()
    item._simulation_manager = SimpleNamespace(get_physics_simulation_view=lambda: view)
    return item


@pytest.mark.parametrize("twist", [(0.5, 0, 0), (-0.5, 0, 0), (0.4, 0, 0.3),
                                    (0.4, 0, -0.3), (0, 0, 0.4), (0, 0, -0.4),
                                    (0, 0.3, 0), (0, -0.3, 0)])
def test_settled_wheel_velocities_reconstruct_requested_body_motion(controller, twist):
    for _ in range(40):
        controller.apply(*twist, 0.05)
    vx, vy, wz = twist
    assert controller.command == pytest.approx(twist)
    for wheel in controller._wheels.values():
        x, y = wheel["pos"]
        angle = math.radians(wheel["steer_target"].value)
        speed = math.radians(wheel["drive_target"].value) * controller.geometry["R"]
        # Independent rigid-body kinematic identity; catches unit/sign/flip errors.
        assert (speed * math.cos(angle), speed * math.sin(angle)) == pytest.approx((vx - wz*y, vy + wz*x), abs=1e-7)
        assert abs(wheel["steer_target"].value) <= 90
    assert controller.stage.edit_layer == "root"


def test_acceleration_braking_and_dt_spike_are_bounded(controller):
    assert controller.apply(1.0, 0, 0, 10.0)[0] == pytest.approx(0.2)
    for _ in range(4):
        controller.apply(1.0, 0, 0, 0.1)
    assert controller.command[0] == pytest.approx(1.0)
    assert controller.apply(0, 0, 0, 0.1)[0] == pytest.approx(0.7)
    assert controller.apply(0, 0, 0, 0.1, handbrake=True)[0] == pytest.approx(0.25)


def test_reverse_does_not_jump_from_forward_to_backward(controller):
    for _ in range(5):
        controller.apply(1, 0, 0, 0.1)
    assert controller.apply(-1, 0, 0, 0.1)[0] == pytest.approx(0.7)
    assert all(w["drive_target"].value > 0 for w in controller._wheels.values())


def test_turn_command_limits_steering_rate_and_initial_drive(controller):
    controller.apply(0, 0.5, 0, 0.01)
    for wheel in controller._wheels.values():
        assert wheel["steer_target"].value == pytest.approx(2.4)
        ungated = math.degrees(controller.command[1] / controller.geometry["R"])
        assert 0 < wheel["drive_target"].value < ungated * 0.1


@pytest.mark.parametrize("index,bad", [(i, bad) for i in range(4)
                                      for bad in (True, None, "0", float("nan"), float("inf"), -float("inf"), 10**400)])
def test_invalid_numbers_cannot_author_any_velocity(controller, index, bad):
    args = [0.1, 0.0, 0.0, 0.05]
    args[index] = bad
    with pytest.raises(ValueError):
        controller.apply(*args)
    assert controller.command == [0, 0, 0]
    assert all(not w["drive_target"].writes for w in controller._wheels.values())


@pytest.mark.parametrize("args", [(2.51, 0, 0, 0.1), (0, 1.51, 0, 0.1),
                                  (0, 0, 1.01, 0.1), (0, 0, 0, -0.1)])
def test_out_of_range_commands_fail_before_writes(controller, args):
    with pytest.raises(ValueError):
        controller.apply(*args)
    assert all(not w["drive_target"].writes for w in controller._wheels.values())


def test_non_boolean_handbrake_is_rejected(controller):
    with pytest.raises(ValueError):
        controller.apply(0, 0, 0, 0.1, handbrake="false")


def test_zero_clears_velocity_without_claiming_vehicle_is_stationary(controller):
    controller.apply(0, 0, 0.4, 0.1)
    angles = dict(controller._steer_now)
    controller.zero()
    assert controller.command == [0, 0, 0] and controller._steer_now == angles
    assert all(w["drive_target"].value == 0 for w in controller._wheels.values())
    controller._simulation_manager.get_physics_simulation_view().body.velocity[0] = 0.2
    assert controller.telemetry(True)["speed_mps"] == pytest.approx(0.2)


def test_zero_attempts_remaining_wheels_after_one_write_fails(controller):
    controller.apply(1, 0, 0, 0.1)
    wheels = list(controller._wheels.values())
    def fail(value):
        raise RuntimeError("expired attribute")
    wheels[0]["drive_target"].Set = fail
    with pytest.raises(RuntimeError, match="expired attribute"):
        controller.zero()
    assert controller.command == [0, 0, 0]
    assert all(w["drive_target"].value == 0 for w in wheels[1:])


def test_tensor_world_to_body_conversion_uses_actual_orientation(controller):
    body = controller._simulation_manager.get_physics_simulation_view().body
    body.pose[3:] = [0, 0, math.sqrt(0.5), math.sqrt(0.5)]
    body.velocity = [0, 0.6, 0, 0, 0, -0.3]
    result = controller.telemetry(True)
    assert result["measurement_valid"] is True
    assert result["position_m"] == [3, 4, 5]
    assert result["yaw_rad"] == pytest.approx(math.pi / 2)
    assert result["roll_rad"] == pytest.approx(0)
    assert result["pitch_rad"] == pytest.approx(0)
    assert result["linear_velocity_body_mps"] == pytest.approx([0.6, 0, 0])
    assert result["linear_velocity_world_mps"] == [0, 0.6, 0]
    assert result["yaw_rate_rps"] == pytest.approx(-0.3)
    assert result["command"] == [0, 0, 0]


@pytest.mark.parametrize("axis,key", [(0, "roll_rad"), (1, "pitch_rad"), (2, "yaw_rad")])
def test_tilt_and_yaw_are_measured_from_xyzw_quaternion(controller, axis, key):
    body = controller._simulation_manager.get_physics_simulation_view().body
    body.pose[3:] = [0, 0, 0, math.cos(math.pi / 12)]
    body.pose[3 + axis] = math.sin(math.pi / 12)
    assert controller.telemetry(True)[key] == pytest.approx(math.pi / 6)


def test_stopped_state_never_returns_fake_pose_or_touches_physics(controller):
    def unavailable():
        raise AssertionError("Stopped telemetry must not request a physics view")
    controller._simulation_manager.get_physics_simulation_view = unavailable
    result = controller.telemetry(False)
    assert result["measurement_valid"] is False
    assert "position_m" not in result and "speed_mps" not in result


def test_reset_and_new_simulation_view_rebind_measurements(controller):
    first = controller._simulation_manager.get_physics_simulation_view()
    controller.telemetry(True)
    controller.telemetry(True)
    assert first.bindings == [controller.chassis_path]
    controller.reset_physics()
    assert controller._tensor_body is None
    controller.telemetry(True)
    assert len(first.bindings) == 2
    second = SimulationView()
    second.body.pose[0] = 100
    controller._simulation_manager.get_physics_simulation_view = lambda: second
    assert controller.telemetry(True)["position_m"][0] == 100
    assert second.bindings == [controller.chassis_path]


@pytest.mark.parametrize("failure", ["uninitialized", "missing_body", "bad_shape", "nan_pose", "bad_quaternion"])
def test_invalid_physics_data_is_never_reported_as_success(controller, failure):
    sim = controller._simulation_manager.get_physics_simulation_view()
    if failure == "uninitialized":
        controller._simulation_manager.get_physics_simulation_view = lambda: None
    elif failure == "missing_body":
        sim.body.count = 0
    elif failure == "bad_shape":
        sim.body.pose = [1, 2]
    elif failure == "nan_pose":
        sim.body.pose[0] = float("nan")
    else:
        sim.body.pose[3:] = [0, 0, 0, 0]
    with pytest.raises((ValueError, RuntimeError)):
        controller.telemetry(True)


def test_returned_command_geometry_and_wheels_cannot_change_control_state(controller):
    result = controller.telemetry(False)
    result["command"][0] = 10
    result["geometry"]["R"] = 100
    result["wheels"]["fl"]["position_body_m"][0] = 100
    assert controller.command == [0, 0, 0]
    assert controller.geometry["R"] == 0.35
    assert controller._wheels["fl"]["pos"][0] == 1.1


def test_canonical_import_is_inert_and_compatibility_import_is_same_class():
    assert RV1Controller is CanonicalController
    result = subprocess.run(
        [sys.executable, "-c", "import sys; from robots.rv1_tune_control import RV1Controller; "
         "assert not any(n == 'omni' or n.startswith('omni.') or n == 'rclpy' or n == 'carb' "
         "for n in sys.modules)"],
        cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert result.stdout == ""


@pytest.mark.parametrize("yaw", [-0.500001, 0.500001, -0.8, 0.8])
def test_canonical_actuator_rejects_unaccepted_yaw_envelope(controller, yaw):
    with pytest.raises(ValueError, match="velocity limits"):
        controller.apply(0, 0, yaw, 0.1)
    assert all(not wheel["drive_target"].writes for wheel in controller._wheels.values())
    assert controller.limits["wz_rps"] == 0.5


def test_standalone_source_requires_explicit_navigation_runtime(tmp_path):
    with pytest.raises(ValueError, match="runtime_path"):
        start_ros2(tmp_path)


def test_standalone_uses_physics_clock_and_never_implicitly_arms(controller):
    calls = []
    nav = SimpleNamespace(start=lambda: calls.append("start"), close=lambda: calls.append("close"),
                          update=lambda t, **kw: calls.append((t, kw)),
                          arm=lambda: calls.append("arm"), status=lambda: {"armed": False})
    timeline = SimpleNamespace(playing=False)
    timeline.is_playing = lambda: timeline.playing
    # A caller that accidentally uses animation time fails this regression.
    timeline.get_current_time = lambda: (_ for _ in ()).throw(AssertionError("animation clock"))
    sub = SimpleNamespace(unsubscribe=lambda: calls.append("unsubscribe"))
    app = SimpleNamespace(get_update_event_stream=lambda:
                          SimpleNamespace(create_subscription_to_pop=lambda *a, **kw: sub))
    usd = SimpleNamespace(get_stage=lambda: controller.stage)
    manager = SimpleNamespace(get_simulation_time=lambda: 12.5)
    session = _Ros2Session(controller, nav, app, timeline, usd, manager)
    assert calls == ["start"]
    with pytest.raises(RuntimeError, match="playback"):
        session.arm()
    session._update(None)
    assert calls[-1] == (12.5, {"playing": False})
    timeline.playing = True
    session.arm()
    assert calls[-1] == "arm"
    session._update(None)
    assert calls[-1] == (12.5, {"playing": True})
    session.close()
    session.close()
    assert calls[-2:] == ["unsubscribe", "close"]
    assert calls.count("close") == 1


def test_standalone_stage_change_closes_ros_session(controller):
    calls = []
    nav = SimpleNamespace(start=lambda: None, close=lambda: calls.append("close"))
    sub = SimpleNamespace(unsubscribe=lambda: calls.append("unsubscribe"))
    app = SimpleNamespace(get_update_event_stream=lambda:
                          SimpleNamespace(create_subscription_to_pop=lambda *a, **kw: sub))
    session = _Ros2Session(controller, nav, app, SimpleNamespace(),
                          SimpleNamespace(get_stage=lambda: object()), SimpleNamespace())
    with pytest.raises(RuntimeError, match="stage changed"):
        session._update(None)
    assert calls == ["unsubscribe", "close"]
