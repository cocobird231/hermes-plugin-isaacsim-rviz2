"""Worker protocol/state tests only; these do not replace real Isaac acceptance."""

import importlib.util
import json
from pathlib import Path
import queue
import sys
import threading
import time
from types import SimpleNamespace
import uuid

import pytest


_SPEC = importlib.util.spec_from_file_location(
    "isaac_plugin_runtime_tests", Path(__file__).resolve().parents[1] / "runtime.py"
)
runtime = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(runtime)


class FakeTask:
    def __init__(self):
        self.completed = False
        self.cancelled = False
        self.outcome = (True, "")

    def done(self):
        return self.completed or self.cancelled

    def result(self):
        assert self.completed and not self.cancelled
        return self.outcome

    def cancel(self):
        self.cancelled = True


class FakeApp:
    def __init__(self):
        self.task = FakeTask()
        self.scheduled = []

    def run_coroutine(self, coroutine, run_until_complete=True):
        assert run_until_complete is False, "USD loading must not block the Kit loop"
        self.scheduled.append(coroutine)
        return self.task


class FakeTimeline:
    def __init__(self):
        self.mode = "stopped"
        self.current_time = 0.0
        self.play_calls = self.stop_calls = 0

    def is_playing(self):
        return self.mode == "playing"

    def is_stopped(self):
        return self.mode == "stopped"

    def get_current_time(self):
        return self.current_time

    def play(self):
        self.play_calls += 1
        self.mode = "playing"

    def stop(self):
        self.stop_calls += 1
        self.mode = "stopped"


class FakeSimulationManager:
    def __init__(self):
        self.current_time = 0.0
        self.steps = 0

    def get_simulation_time(self):
        return self.current_time

    def get_num_physics_steps(self):
        return self.steps


class FakeUsd:
    def __init__(self):
        self.path = None
        self.opened = []
        self.loading_files = 0

    def open_stage_async(self, path):
        self.opened.append(path)
        return ("open_stage_async", path)

    def get_stage(self):
        return SimpleNamespace(GetRootLayer=lambda: SimpleNamespace(realPath=self.path)) if self.path else None

    def get_stage_loading_status(self):
        return ("loading" if self.loading_files else "", 0, self.loading_files)


@pytest.fixture
def worker(tmp_path):
    instance = runtime.Worker(tmp_path, str(uuid.uuid4()))
    instance.app = FakeApp()
    instance.timeline = FakeTimeline()
    instance.simulation_manager = FakeSimulationManager()
    instance.usd = FakeUsd()
    instance.change(phase="ready")
    return instance


@pytest.fixture
def usd_path(tmp_path):
    path = tmp_path / "scene.usda"
    path.write_text('#usda 1.0\ndef Xform "World" {}\n', encoding="utf-8")
    return str(path)


def dispatch(worker, action, request_id=None, **fields):
    responses = queue.Queue(maxsize=1)
    request = {"action": action, "session_id": worker.state["session_id"],
               "request_id": request_id or str(uuid.uuid4()), **fields}
    worker.dispatch((request, responses, threading.Event(), time.monotonic() + 30))
    return responses


def tick(worker, simulation_time=None, *, animation_time=None, physics_steps=None):
    if simulation_time is not None:
        if physics_steps is None and simulation_time > worker.simulation_manager.current_time:
            worker.simulation_manager.steps += 1
        worker.simulation_manager.current_time = simulation_time
    if physics_steps is not None:
        worker.simulation_manager.steps = physics_steps
    if animation_time is not None:
        worker.timeline.current_time = animation_time
    worker.sample()
    worker.advance()


def test_stop_waits_for_stable_time_after_real_stop_call(worker):
    worker.timeline.mode = "playing"
    responses = dispatch(worker, "stop")
    assert worker.timeline.stop_calls == 1
    accepted = responses.get_nowait()
    assert accepted["success"] is True and accepted["accepted"] is True
    assert accepted["state"]["operation"]["status"] == "running"

    assert responses.empty()

    tick(worker, 12.0)
    tick(worker, 12.1)
    tick(worker, 12.2)
    assert responses.empty(), "stop must not succeed while simulation time is still changing"
    assert worker.state["operation"]["status"] == "running"

    tick(worker, 12.2)
    result = dispatch(worker, "status").get_nowait()
    assert result["state"]["timeline"] == "stopped"
    assert result["state"]["simulation_time"] == 12.2
    assert result["state"]["operation"]["status"] == "succeeded"
    assert responses.empty(), "Completion must not send a second response to the accepted request"
    assert accepted["state"]["operation"]["status"] == "running"


@pytest.mark.parametrize("mode", ["playing", "paused"])
def test_insert_requires_stopped_timeline(worker, mode):
    worker.timeline.mode = mode
    result = dispatch(worker, "rv1_insert").get_nowait()
    assert result["error"]["code"] == "STOP_REQUIRED"
    assert not worker.usd.opened and worker.pending is None


def test_insert_invalid_scene_does_not_open_or_modify_stage(worker, monkeypatch):
    def reject(stage):
        raise ValueError("Wrong scene")
    monkeypatch.setitem(sys.modules, "rv1.gongguan", SimpleNamespace(validate_target=reject))
    result = dispatch(worker, "rv1_insert").get_nowait()
    assert not result["success"]
    assert "Wrong scene" in result["error"]["message"]
    assert not worker.usd.opened and worker.pending is None


def test_insert_acknowledges_before_composition_and_never_opens_stage(worker, monkeypatch):
    inserted = []
    def insert(stage):
        inserted.append(stage)
        return {"camera_path": "/World/AgentExperiment/OverviewCamera"}
    monkeypatch.setitem(sys.modules, "rv1.gongguan", SimpleNamespace(validate_target=lambda stage: None, insert_rv1=insert))
    result = dispatch(worker, "rv1_insert").get_nowait()
    assert result["accepted"] and worker.state["phase"] == "loading"
    assert not inserted and not worker.usd.opened
    tick(worker)
    tick(worker)
    assert len(inserted) == 1 and not worker.usd.opened
    assert worker.pending["inserted"] and worker.state["operation"]["status"] == "running"


@pytest.fixture
def inserted_scene(worker, monkeypatch, usd_path):
    """Model a retained stage; real USD composition is checked separately."""
    stage = SimpleNamespace(GetRootLayer=lambda: SimpleNamespace(realPath=usd_path), inserted=False)
    monkeypatch.setattr(worker.usd, "get_stage", lambda: stage)
    setup = {"map_path": usd_path, "camera_path": "/World/AgentExperiment/OverviewCamera",
             "edit_layer": "session", "source_files_saved": False}
    events = []

    def validate(target):
        assert target is stage
        if stage.inserted:
            raise ValueError("AgentExperiment already exists; reload Gongguan before inserting again")

    def compose(target):
        validate(target)
        stage.inserted = True
        events.append("compose")
        return dict(setup)

    monkeypatch.setitem(sys.modules, "rv1.gongguan", SimpleNamespace(validate_target=validate, insert_rv1=compose))
    worker.sample()
    return stage, setup, events


def test_insert_finishes_only_after_controller_binding_and_keeps_existing_stage(worker, monkeypatch, inserted_scene):
    stage, setup, events = inserted_scene

    def telemetry(playing):
        assert not playing
        events.append("telemetry")
        return {"measurement_valid": False, "chassis_path": "/World/AgentExperiment/RV1/rv1/base_link/chassis_link"}

    controller = SimpleNamespace(stage=stage, telemetry=telemetry)

    def bind(target, simulation_manager):
        assert target is stage and simulation_manager is worker.simulation_manager
        assert worker.state["operation"]["status"] == "running"
        events.append("bind")
        return controller

    def viewport():
        assert worker.rv1 is controller and worker.state["operation"]["status"] == "running"
        events.append("viewport")
        return SimpleNamespace()

    monkeypatch.setitem(sys.modules, "rv1.controller", SimpleNamespace(RV1Controller=bind))
    monkeypatch.setitem(sys.modules, "omni.kit.viewport.utility", SimpleNamespace(get_active_viewport=viewport))
    accepted = dispatch(worker, "rv1_insert", request_id="insert-success").get_nowait()
    assert accepted["accepted"] and accepted["state"]["operation"]["status"] == "running"
    tick(worker)
    tick(worker)
    assert events == ["compose"] and worker.pending is not None
    assert worker.state["operation"]["status"] == "running"
    tick(worker)
    tick(worker)
    assert events == ["compose", "bind", "viewport", "telemetry"]
    assert worker.state["operation"] == {"id": "insert-success", "action": "rv1_insert", "status": "succeeded"}
    assert worker.state["phase"] == "ready" and worker.pending is None
    assert worker.usd.get_stage() is stage and worker.state["stage_path"] == setup["map_path"]
    assert worker.rv1 is controller and worker.state["rv1"]["chassis_path"].startswith("/World/AgentExperiment/")
    assert not worker.usd.opened and not worker.app.scheduled
    assert worker.timeline.play_calls == worker.timeline.stop_calls == 0
    assert worker.cache["insert-success"][1]["success"] is True


def test_insert_controller_failure_is_failed_and_requires_reload_of_retained_composition(worker, monkeypatch, inserted_scene):
    stage, setup, events = inserted_scene

    def fail_bind(target, simulation_manager):
        assert target is stage and stage.inserted
        events.append("bind-failed")
        raise RuntimeError("RV1 wheel joint is missing")

    monkeypatch.setitem(sys.modules, "rv1.controller", SimpleNamespace(RV1Controller=fail_bind))
    accepted = dispatch(worker, "rv1_insert", request_id="insert-failure").get_nowait()
    assert accepted["accepted"]
    for _ in range(4):
        tick(worker)
    operation = worker.state["operation"]
    assert operation["id"] == "insert-failure" and operation["status"] == "failed"
    assert operation["error"] == {"code": "OPERATION_FAILED", "message": "RV1 wheel joint is missing"}
    assert worker.cache["insert-failure"][1]["success"] is False
    assert worker.pending is None and worker.state["phase"] == "ready" and worker.rv1 is None
    # Runtime must not claim rollback of composition after later binding fails.
    assert stage.inserted and worker.state["scene_setup"] == setup
    assert worker.usd.get_stage() is stage and worker.state["stage_path"] == setup["map_path"]
    rejected = dispatch(worker, "rv1_insert", request_id="retry-without-reload").get_nowait()
    assert not rejected["success"] and "reload Gongguan" in rejected["error"]["message"]
    assert worker.state["operation"] == operation and events == ["compose", "bind-failed"]
    assert not worker.usd.opened and not worker.app.scheduled
    assert worker.timeline.play_calls == worker.timeline.stop_calls == 0


def test_stop_does_not_treat_paused_timeline_as_stopped(worker):
    responses = dispatch(worker, "stop")
    assert responses.get_nowait()["accepted"] is True
    worker.timeline.mode = "paused"
    for _ in range(4):
        tick(worker, 0.0)
    assert responses.empty()
    assert worker.state["operation"]["status"] == "running"
    worker.timeline.mode = "stopped"
    tick(worker, 0.0)
    assert worker.state["operation"]["status"] == "succeeded"
    assert responses.empty()


def test_play_requires_simulation_time_to_advance(worker, usd_path):
    worker.usd.path = usd_path
    worker.simulation_manager.current_time = 1.0
    worker.sample()
    responses = dispatch(worker, "play")
    accepted = responses.get_nowait()
    assert accepted["accepted"] is True
    assert accepted["state"]["operation"]["status"] == "running"
    for _ in range(4):
        tick(worker, 1.0)
    assert worker.timeline.is_playing()
    assert worker.state["operation"]["status"] == "running", "Playing flag alone does not prove simulation advancement"
    tick(worker, 0.5)
    assert worker.state["operation"]["status"] == "running", "Time moving backwards is not advancement"
    tick(worker, 1.016)
    assert worker.state["operation"]["status"] == "succeeded"
    assert responses.empty()


def test_play_succeeds_when_physics_advances_despite_animation_loop(worker, usd_path):
    worker.usd.path = usd_path
    worker.simulation_manager.current_time = 10.0
    worker.simulation_manager.steps = 600
    worker.timeline.current_time = 0.0
    worker.sample()
    accepted = dispatch(worker, "play").get_nowait()
    assert accepted["state"]["operation"]["status"] == "running"
    # A static map can author animation start/end=-1/0 at 24 fps. Its
    # looping animation clock must not hide genuine physics advancement.
    tick(worker, 10.016, animation_time=-1 / 24, physics_steps=601)
    tick(worker, 10.033, animation_time=-1 / 48, physics_steps=602)
    result = dispatch(worker, "status").get_nowait()["state"]
    assert result["operation"]["status"] == "succeeded"
    assert result["simulation_time"] == 10.033
    assert result["physics_steps"] == 602
    assert result["animation_time"] == -1 / 48


def test_play_requires_physics_time_and_steps_not_animation_advancement(worker, usd_path):
    worker.usd.path = usd_path
    worker.sample()
    dispatch(worker, "play").get_nowait()
    for animation_time in range(1, 5):
        tick(worker, 0.0, animation_time=animation_time, physics_steps=0)
    assert worker.state["operation"]["status"] == "running"
    tick(worker, 0.1, animation_time=5.0, physics_steps=0)
    assert worker.state["operation"]["status"] == "running", "Physics steps must advance too"
    tick(worker, 0.2, animation_time=6.0, physics_steps=1)
    assert worker.state["operation"]["status"] == "succeeded"


def test_stop_waits_for_stable_physics_steps_even_when_time_is_stable(worker):
    dispatch(worker, "stop").get_nowait()
    tick(worker, 0.0, physics_steps=1)
    tick(worker, 0.0, physics_steps=2)
    tick(worker, 0.0, physics_steps=3)
    assert worker.state["operation"]["status"] == "running"
    tick(worker, 0.0, physics_steps=3)
    assert worker.state["operation"]["status"] == "succeeded"


def test_stop_cancels_pending_play_and_completes_its_own_operation(worker, usd_path):
    worker.usd.path = usd_path
    worker.sample()
    playing = dispatch(worker, "play", request_id="play-before-stop").get_nowait()
    assert playing["accepted"] is True and worker.timeline.is_playing()
    assert worker.state["operation"]["status"] == "running"

    stopped = dispatch(worker, "stop", request_id="interrupt-play").get_nowait()
    assert stopped["accepted"] is True
    assert stopped["state"]["operation"] == {"id": "interrupt-play", "action": "stop", "status": "running"}
    assert worker.timeline.stop_calls == 1
    cancelled = dispatch(worker, "play", request_id="play-before-stop").get_nowait()
    assert cancelled["success"] is False
    assert cancelled["state"]["operation"]["status"] == "failed"
    assert cancelled["state"]["operation"]["id"] == "play-before-stop"
    assert "Cancelled by stop request" in cancelled["error"]["message"]
    assert worker.timeline.play_calls == 1

    for _ in range(3):
        tick(worker, 0.0, physics_steps=0)
    completed = dispatch(worker, "stop", request_id="interrupt-play").get_nowait()
    assert completed["success"] is True
    assert completed["state"]["operation"] == {"id": "interrupt-play", "action": "stop", "status": "succeeded"}
    assert worker.timeline.stop_calls == 1


def test_stop_does_not_cancel_pending_load(worker, usd_path):
    dispatch(worker, "load", request_id="keep-loading", path=usd_path).get_nowait()
    stopped = dispatch(worker, "stop").get_nowait()
    assert stopped["success"] is False
    assert stopped["error"]["code"] == "BUSY"
    assert worker.state["operation"] == {"id": "keep-loading", "action": "load", "status": "running"}
    assert not worker.app.task.cancelled
    assert worker.timeline.stop_calls == 0


class FakeRV1:
    def __init__(self):
        self.zero_calls = self.reset_calls = 0

    def zero(self):
        self.zero_calls += 1

    def reset_physics(self):
        self.reset_calls += 1
        self.zero()

    def telemetry(self, playing):
        return {"measurement_valid": True, "position_m": [0, 0, 0.8], "yaw_rad": 0.0}


@pytest.mark.parametrize("action", ["rv1_drive", "rv1_brake"])
def test_removed_direct_control_actions_rejected_even_with_live_rv1(worker, action):
    worker.rv1 = FakeRV1()
    worker.timeline.mode = "playing"
    result = dispatch(worker, action).get_nowait()
    assert result["error"]["code"] == "UNKNOWN_ACTION"
    assert worker.pending is None and worker.rv1.zero_calls == 0
    assert worker.timeline.is_playing()


@pytest.mark.parametrize("action", ["stop", "close"])
def test_lifecycle_revokes_navigation_and_clears_targets(worker, action):
    worker.rv1 = FakeRV1()
    worker.timeline.mode = "playing"
    calls = []
    worker.rv1_navigation = SimpleNamespace(close=lambda: calls.append("closed"))
    assert dispatch(worker, action).get_nowait()["accepted"]
    assert calls == ["closed"] and worker.rv1_navigation is None
    assert worker.rv1.zero_calls > 0 and worker.rv1.reset_calls > 0
    assert worker.state["control_owner"] == "stopped"
    assert worker.timeline.is_stopped()


@pytest.mark.parametrize("action", ["stop", "close"])
def test_failed_navigation_cleanup_cannot_block_timeline_stop(worker, action):
    worker.rv1 = FakeRV1()
    worker.timeline.mode = "playing"
    def invalid():
        raise RuntimeError("expired adapter target")
    worker.rv1_navigation = SimpleNamespace(close=invalid)
    assert dispatch(worker, action).get_nowait()["accepted"]
    assert worker.rv1_navigation is None and worker.rv1.zero_calls > 0
    assert worker.timeline.is_stopped()


def test_rv1_invalidated_attributes_do_not_prevent_close(worker):
    worker.rv1 = FakeRV1()
    def invalid():
        raise RuntimeError("expired USD attribute")
    worker.rv1.reset_physics = invalid
    assert dispatch(worker, "close").get_nowait()["accepted"]
    assert worker.timeline.is_stopped()


def test_rv1_navigation_measurement_failure_stops_even_when_target_clear_fails(worker):
    worker.rv1 = FakeRV1()
    worker.rv1.stage = object()
    worker.timeline.mode = "playing"
    calls = []
    worker.rv1_navigation = SimpleNamespace(status=lambda: {"armed": True},
                                            close=lambda: calls.append("closed"))
    def invalid(*args):
        raise RuntimeError("expired USD attribute")
    worker.rv1.telemetry = worker.rv1.zero = worker.rv1.reset_physics = invalid
    worker.sample_rv1(worker.rv1.stage)
    assert worker.timeline.is_stopped()
    assert calls == ["closed"] and worker.rv1_navigation is None
    assert worker.state["rv1"]["measurement_valid"] is False
    assert worker.state["control_owner"] == "stopped"


def test_replaced_stage_detaches_rv1_without_secondary_attribute_error(worker):
    worker.rv1 = FakeRV1()
    worker.rv1.stage = object()
    worker.timeline.mode = "playing"
    worker.sample_rv1(object())
    assert worker.rv1 is None and worker.timeline.is_stopped()
    assert "stage was replaced" in worker.state["rv1"]["error"]


@pytest.mark.parametrize("action", ["play", "stop"])
def test_timeline_operations_allow_shader_compilation_before_completion(worker, usd_path, action):
    worker.usd.path = usd_path
    worker.sample()
    responses = dispatch(worker, action)
    assert responses.get_nowait()["accepted"] is True
    # Simulate an app.update delayed well beyond the socket's 30-second wait.
    worker.pending["started"] = time.monotonic() - 60
    tick(worker, 0.0)
    tick(worker, 0.0)
    assert worker.state["operation"]["status"] == "running"
    tick(worker, 0.016 if action == "play" else 0.0)
    assert worker.state["operation"]["status"] == "succeeded"
    assert responses.empty()


@pytest.mark.parametrize("action", ["play", "stop"])
def test_timeline_operations_still_fail_after_bounded_timeout(worker, usd_path, action):
    worker.usd.path = usd_path
    worker.sample()
    request_id = "timeout-" + action
    responses = dispatch(worker, action, request_id=request_id)
    assert responses.get_nowait()["accepted"] is True
    worker.pending["started"] = time.monotonic() - runtime.OPERATION_TIMEOUT - 1
    worker.advance()
    assert worker.pending is None
    assert worker.state["operation"]["status"] == "failed"
    retry = dispatch(worker, action, request_id=request_id).get_nowait()
    assert retry["success"] is False
    assert "runtime timeout" in retry["error"]["message"]
    assert responses.empty()


@pytest.mark.parametrize("action", ["play", "stop"])
def test_timeline_request_id_retries_do_not_repeat_actions(worker, usd_path, action):
    worker.usd.path = usd_path
    worker.sample()
    request_id = "once-" + action
    first = dispatch(worker, action, request_id=request_id).get_nowait()
    assert first["accepted"] is True
    assert dispatch(worker, action, request_id=request_id).get_nowait() == first
    assert getattr(worker.timeline, action + "_calls") == 1
    conflict = dispatch(worker, "status", request_id=request_id).get_nowait()
    assert conflict["error"]["code"] == "REQUEST_ID_CONFLICT"
    for i in range(3):
        tick(worker, (i + 1) * 0.016 if action == "play" else 0.0)
    completed = dispatch(worker, action, request_id=request_id).get_nowait()
    assert completed["success"] is True
    assert not completed.get("accepted")
    assert completed["state"]["operation"]["status"] == "succeeded"
    assert getattr(worker.timeline, action + "_calls") == 1


def test_load_rejected_while_playing_without_opening_stage(worker, usd_path):
    worker.timeline.mode = "playing"
    result = dispatch(worker, "load", path=usd_path).get_nowait()
    assert result["success"] is False
    assert result["error"]["code"] == "STOP_REQUIRED"
    assert worker.usd.opened == []
    assert worker.app.scheduled == []
    assert worker.pending is None


def test_request_id_retry_never_reopens_stage_and_conflicting_reuse_fails(worker, usd_path):
    first = dispatch(worker, "load", request_id="load-once", path=usd_path).get_nowait()
    retried = dispatch(worker, "load", request_id="load-once", path=usd_path).get_nowait()
    assert retried == first
    assert first["accepted"] is True
    assert worker.usd.opened == [usd_path]

    conflict = dispatch(worker, "stop", request_id="load-once").get_nowait()
    assert conflict["error"]["code"] == "REQUEST_ID_CONFLICT"
    assert worker.timeline.stop_calls == 0

    worker.app.task.completed = True
    worker.usd.path = usd_path
    tick(worker)
    tick(worker)
    completed = dispatch(worker, "load", request_id="load-once", path=usd_path).get_nowait()
    assert completed["success"] is True
    assert completed["state"]["operation"]["status"] == "succeeded"
    assert worker.usd.opened == [usd_path]


def test_load_waits_for_task_and_remaining_assets_then_verifies_actual_root(worker, usd_path):
    accepted = dispatch(worker, "load", path=usd_path).get_nowait()
    assert accepted["accepted"] is True
    assert accepted["state"]["phase"] == "loading"
    worker.usd.path = usd_path
    tick(worker)
    tick(worker)
    assert worker.state["operation"]["status"] == "running"

    worker.app.task.completed = True
    worker.usd.loading_files = 2
    tick(worker)
    assert worker.state["phase"] == "loading"
    worker.usd.loading_files = 0
    tick(worker)
    assert worker.state["phase"] == "ready"
    assert worker.state["stage_path"] == usd_path
    assert worker.state["operation"]["status"] == "succeeded"
    persisted = json.loads((worker.run_dir / "state.json").read_text())
    assert persisted["operation"]["status"] == "succeeded"


def test_load_reports_failure_when_actual_root_differs(worker, usd_path, tmp_path):
    dispatch(worker, "load", path=usd_path).get_nowait()
    worker.app.task.completed = True
    worker.usd.path = str(tmp_path / "unexpected.usda")
    tick(worker)
    tick(worker)
    operation = worker.state["operation"]
    assert operation["status"] == "failed"
    assert "root layer" in operation["error"]["message"]
    assert worker.pending is None
    assert worker.state["stage_path"] == worker.usd.path


def test_load_failure_returned_by_usd_is_not_reported_as_success(worker, usd_path):
    dispatch(worker, "load", path=usd_path).get_nowait()
    worker.app.task.completed = True
    worker.app.task.outcome = (False, "USD parse failed")
    tick(worker)
    tick(worker)
    assert worker.state["operation"]["status"] == "failed"
    assert "USD parse failed" in worker.state["operation"]["error"]["message"]


def test_load_timeout_cancels_task_and_requires_close_before_new_mutations(worker, usd_path):
    dispatch(worker, "load", path=usd_path).get_nowait()
    worker.pending["started"] = time.monotonic() - runtime.OPERATION_TIMEOUT - 1
    worker.advance()
    assert worker.app.task.cancelled is True
    assert worker.pending is None
    assert worker.state["phase"] == "error"
    assert worker.state["operation"]["status"] == "failed"
    assert worker.state["error"]["code"] == "LOAD_TIMEOUT"
    assert dispatch(worker, "load", path=usd_path).get_nowait()["error"]["code"] == "BUSY"
    assert dispatch(worker, "status").get_nowait()["success"] is True
    assert dispatch(worker, "close").get_nowait()["accepted"] is True
    assert worker.usd.opened == [usd_path]


def test_close_cancels_pending_load_and_keeps_cancelled_request_result(worker, usd_path):
    dispatch(worker, "load", request_id="cancel-me", path=usd_path).get_nowait()
    result = dispatch(worker, "close", request_id="close-session").get_nowait()
    assert result["success"] is True
    assert result["accepted"] is True
    assert result["state"]["phase"] == "closing"
    assert result["state"]["operation"]["action"] == "close"
    assert worker.app.task.cancelled is True
    assert worker.pending is None
    assert worker.timeline.stop_calls == 1
    cancelled = dispatch(worker, "load", request_id="cancel-me", path=usd_path).get_nowait()
    assert cancelled["success"] is False
    assert cancelled["state"]["operation"]["status"] == "failed"
    assert "Cancelled by close" in cancelled["error"]["message"]
    assert worker.usd.opened == [usd_path]


@pytest.mark.parametrize("action", ["play", "stop"])
def test_close_cancels_accepted_timeline_request_and_keeps_result(worker, usd_path, action):
    worker.usd.path = usd_path
    worker.sample()
    request_id = "pending-" + action
    responses = dispatch(worker, action, request_id=request_id)
    accepted = responses.get_nowait()
    assert accepted["accepted"] is True
    assert accepted["state"]["operation"]["status"] == "running"
    close = dispatch(worker, "close").get_nowait()
    assert close["accepted"] is True
    assert close["state"]["phase"] == "closing"
    assert worker.pending is None
    cancelled = dispatch(worker, action, request_id=request_id).get_nowait()
    assert cancelled["success"] is False
    assert cancelled["state"]["operation"]["id"] == request_id
    assert cancelled["state"]["operation"]["status"] == "failed"
    assert "Cancelled by close" in cancelled["error"]["message"]
    assert responses.empty()


def test_other_sessions_cannot_mutate_this_worker(worker, usd_path):
    result = dispatch(worker, "load", session_id=str(uuid.uuid4()), path=usd_path).get_nowait()
    assert result["error"]["code"] == "SESSION_MISMATCH"
    assert worker.usd.opened == []
    assert worker.pending is None
