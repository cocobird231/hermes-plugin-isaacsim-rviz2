"""RV1 Nav2 actuator: validated swerve response, with no manual motion interface.

Importing this module has no Isaac, ROS, callback, timeline or keyboard side
effects. RV1Controller is used by the plugin's RV1Navigation /cmd_vel adapter.
Its caller supplies elapsed SimulationManager physics time (not animation time).
USD drive targets and gains are authored only in the session layer; live pose
and speed come from PhysX tensors.

The six drive/steer gains and all slew rates match the previously accepted
controller. The accepted yaw envelope is 0.5 rad/s; no claim of newly optimized
mechanical gains is made. start_ros2() is an explicit standalone integration
entry using the same RV1Navigation implementation from an installed plugin.
"""
import math
from types import MappingProxyType


PARAMS = MappingProxyType({
    "V_MAX": 2.5, "VY_MAX": 1.5, "WZ_MAX": 0.5, "A_Y_MAX": 3.92, "V_EPS": 0.05,
    "A_ACCEL": 2.0, "A_BRAKE": 3.0, "A_BRAKE_HAND": 4.5, "WZ_ACCEL": 2.0,
    "STEER_ANGLE_MAX": 90.0, "STEER_RATE": 240.0, "FLIP_HYST": 2.0,
    "DRIVE_STIFFNESS": 0.0, "DRIVE_DAMPING": 15.0, "DRIVE_MAX_FORCE": 1200.0,
    "STEER_STIFFNESS": 100.0, "STEER_DAMPING": 3.0, "STEER_MAX_FORCE": 500.0,
    "DT_SIM_MAX": 0.1,
    "ROOT_FALLBACKS": ("/World/rv1/base_link", "/Root/rv1/base_link"),
})


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        value = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return value


class RV1Controller:
    def __init__(self, stage, simulation_manager):
        from pxr import Usd, UsdGeom, UsdPhysics
        import omni.timeline

        if not omni.timeline.get_timeline_interface().is_stopped():
            raise RuntimeError("Stop the simulation before initializing RV1 control")
        if not stage or abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) > 1e-6:
            raise ValueError("RV1 control requires metersPerUnit=1")
        if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
            raise ValueError("RV1 control requires a Z-up stage")
        self._stage, self._simulation_manager = stage, simulation_manager
        self._Usd, self._UsdGeom, self._UsdPhysics = Usd, UsdGeom, UsdPhysics
        self._command = [0.0, 0.0, 0.0]
        self._tensor_sim = self._tensor_body = None
        self._wheels, chassis, self._geometry = self._discover()
        self._chassis_path = str(chassis.GetPath())
        self._steer_now, self._drive_now, self._flipped = {}, {}, {}
        self._init_write_gains()
        self.zero()

    @property
    def stage(self):
        return self._stage

    @property
    def chassis_path(self):
        return self._chassis_path

    @property
    def geometry(self):
        return dict(self._geometry)

    @property
    def limits(self):
        return {"vx_mps": PARAMS["V_MAX"], "vy_mps": PARAMS["VY_MAX"],
                "wz_rps": PARAMS["WZ_MAX"], "original_controller_wz_rps": 1.0}

    @property
    def command(self):
        return list(self._command)

    def _edit(self):
        return self._Usd.EditContext(self.stage, self.stage.GetSessionLayer())

    def _require_unscaled(self, prim):
        transform = self._UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(self._Usd.TimeCode.Default())
        rows = [transform.GetRow3(i) for i in range(3)]
        # Rotation/translation is allowed; scale, reflection and shear are not.
        for i in range(3):
            for j in range(3):
                product = sum(float(rows[i][k]) * float(rows[j][k]) for k in range(3))
                if not math.isfinite(product) or abs(product - (1.0 if i == j else 0.0)) > 1e-5:
                    raise ValueError(f"Unsupported scale or shear at {prim.GetPath()}")
        if not math.isclose(float(transform.GetDeterminant()), 1.0, abs_tol=1e-5):
            raise ValueError(f"Unsupported reflected transform at {prim.GetPath()}")

    @staticmethod
    def _child_joint(body, type_name):
        return next((p for p in body.GetChildren() if p.GetTypeName() == type_name), None)

    def _discover(self):
        stage, physics = self.stage, self._UsdPhysics
        root = None
        candidates = [stage.GetPrimAtPath(path) for path in PARAMS["ROOT_FALLBACKS"]]
        candidates.extend(p for p in stage.Traverse()
                          if p.HasAPI(physics.ArticulationRootAPI) and p.GetName() == "base_link")
        for candidate in candidates:
            if candidate.IsValid() and any(c.GetName().endswith("_wheel_shock") for c in candidate.GetChildren()):
                root = candidate
                break
        if root is None:
            raise ValueError("RV1 base_link with wheel joints was not found")
        self._require_unscaled(root)
        root_path = root.GetPath()
        chassis = stage.GetPrimAtPath(root_path.AppendChild("chassis_link"))
        if not chassis.IsValid() or not chassis.HasAPI(physics.RigidBodyAPI):
            raise ValueError("RV1 chassis_link must be a valid rigid body")
        self._require_unscaled(chassis)
        wheels, radii = {}, []
        for shock in root.GetChildren():
            if not shock.GetName().endswith("_wheel_shock"):
                continue
            name = shock.GetName()[:-len("_wheel_shock")]
            hub = stage.GetPrimAtPath(root_path.AppendChild(name + "_wheel_hub"))
            tire = stage.GetPrimAtPath(root_path.AppendChild(name + "_wheel_tire"))
            if not hub.IsValid() or not tire.IsValid():
                raise ValueError(f"RV1 {name} is missing a wheel hub or tire")
            for body in (shock, hub, tire):
                self._require_unscaled(body)
            shock_joint = self._child_joint(shock, "PhysicsPrismaticJoint")
            steer_joint = self._child_joint(hub, "PhysicsRevoluteJoint")
            drive_joint = self._child_joint(tire, "PhysicsRevoluteJoint")
            if not all((shock_joint, steer_joint, drive_joint)):
                raise ValueError(f"RV1 {name} is missing a shock, steer or drive joint")
            pos = shock_joint.GetAttribute("physics:localPos0").Get()
            if pos is None or len(pos) != 3:
                raise ValueError(f"RV1 {name} has no valid shock position")
            pos = tuple(_number(float(v), "wheel position") for v in pos)
            collision = stage.GetPrimAtPath(tire.GetPath().AppendChild("collision"))
            if collision.IsValid() and collision.GetTypeName() == "Cylinder":
                self._require_unscaled(collision)
                radius = _number(collision.GetAttribute("radius").Get(), "wheel radius")
            else:
                visual = stage.GetPrimAtPath(tire.GetPath().AppendChild("Cylinder"))
                scale = visual.GetAttribute("xformOp:scale").Get() if visual.IsValid() else None
                if scale is None or len(scale) != 3:
                    raise ValueError(f"RV1 {name} wheel radius could not be derived")
                sx, sy = _number(float(scale[0]), "wheel scale"), _number(float(scale[1]), "wheel scale")
                if sx <= 0 or not math.isclose(sx, sy, rel_tol=1e-5):
                    raise ValueError(f"RV1 {name} has unsupported tire scale")
                radius = sx * 0.5
            wheels[name] = {"pos": pos[:2], "steer": steer_joint, "drive": drive_joint}
            radii.append(radius)
        if len(wheels) != 4:
            raise ValueError("RV1 control requires exactly four complete wheel corners")
        xs, ys = [w["pos"][0] for w in wheels.values()], [w["pos"][1] for w in wheels.values()]
        geometry = {"R": radii[0], "L": max(xs) - min(xs), "W": max(ys) - min(ys)}
        if (not all(0.05 <= r <= 1.0 and math.isclose(r, radii[0], rel_tol=1e-4) for r in radii)
                or not 0.5 <= geometry["L"] <= 5.0 or not 0.5 <= geometry["W"] <= 5.0):
            raise ValueError(f"RV1 geometry failed validation: {geometry}")
        return wheels, chassis, geometry

    def _init_write_gains(self):
        with self._edit():
            for name, wheel in self._wheels.items():
                for role, prefix in (("steer", "STEER"), ("drive", "DRIVE")):
                    drive = self._UsdPhysics.DriveAPI.Get(wheel[role], "angular")
                    if not drive:
                        drive = self._UsdPhysics.DriveAPI.Apply(wheel[role], "angular")
                    drive.GetTypeAttr().Set("force")
                    drive.GetStiffnessAttr().Set(PARAMS[prefix + "_STIFFNESS"])
                    drive.GetDampingAttr().Set(PARAMS[prefix + "_DAMPING"])
                    drive.GetMaxForceAttr().Set(PARAMS[prefix + "_MAX_FORCE"])
                    if role == "steer":
                        attr = drive.GetTargetPositionAttr()
                        if not attr:
                            attr = drive.CreateTargetPositionAttr(0.0)
                        old = attr.Get()
                        old = 0.0 if old is None else _number(old, "steering target")
                        wheel["steer_target"] = attr
                        self._steer_now[name] = max(-90.0, min(90.0, old))
                    else:
                        attr = drive.GetTargetVelocityAttr()
                        wheel["drive_target"] = attr if attr else drive.CreateTargetVelocityAttr(0.0)
                self._flipped[name] = False
                self._drive_now[name] = 0.0

    def _envelope(self, vx, vy, wz):
        p, geometry = PARAMS, self._geometry
        vx = max(-p["V_MAX"], min(p["V_MAX"], vx))
        vy = max(-p["VY_MAX"], min(p["VY_MAX"], vy))
        wz_cap = min(p["WZ_MAX"], p["A_Y_MAX"] / max(math.hypot(vx, vy), p["V_EPS"]))
        wz = max(-wz_cap, min(wz_cap, wz))
        limit = p["V_MAX"] + p["WZ_MAX"] * math.hypot(geometry["L"] / 2.0, geometry["W"] / 2.0)
        maximum = max(math.hypot(vx - wz * w["pos"][1], vy + wz * w["pos"][0]) for w in self._wheels.values())
        if maximum > limit:
            scale = limit / maximum
            return vx * scale, vy * scale, wz * scale
        return vx, vy, wz

    @staticmethod
    def _slew_linear(current, target, dt, handbrake):
        rate = PARAMS["A_BRAKE_HAND"] if handbrake else (
            PARAMS["A_BRAKE"] if abs(target) < abs(current) or target * current < 0.0 else PARAMS["A_ACCEL"])
        step = rate * dt
        return current + max(-step, min(step, target - current))

    def _wheel_targets(self, name, vx, vy, wz):
        x, y = self._wheels[name]["pos"]
        wx, wy = vx - wz * y, vy + wz * x
        speed = math.hypot(wx, wy)
        if speed < 1e-4:
            return None, 0.0
        theta, omega = math.degrees(math.atan2(wy, wx)), math.degrees(speed / self._geometry["R"])
        if abs(theta) > 90.0 + PARAMS["FLIP_HYST"]:
            self._flipped[name] = True
        elif abs(theta) < 90.0 - PARAMS["FLIP_HYST"]:
            self._flipped[name] = False
        if self._flipped[name]:
            theta -= math.copysign(180.0, theta)
            omega = -omega
        return max(-PARAMS["STEER_ANGLE_MAX"], min(PARAMS["STEER_ANGLE_MAX"], theta)), omega

    def apply(self, vx, vy, wz, dt, handbrake=False):
        values = [_number(v, name) for v, name in zip((vx, vy, wz), ("vx", "vy", "wz"))]
        dt = _number(dt, "dt")
        if not isinstance(handbrake, bool) or dt < 0:
            raise ValueError("handbrake must be boolean and dt must be nonnegative")
        if any(abs(v) > limit for v, limit in zip(values, (PARAMS["V_MAX"], PARAMS["VY_MAX"], PARAMS["WZ_MAX"]))):
            raise ValueError("RV1 command exceeds velocity limits")
        dt = min(dt, PARAMS["DT_SIM_MAX"])
        target = self._envelope(*values)
        self._command[0] = self._slew_linear(self._command[0], target[0], dt, handbrake)
        self._command[1] = self._slew_linear(self._command[1], target[1], dt, handbrake)
        step = PARAMS["WZ_ACCEL"] * dt
        self._command[2] += max(-step, min(step, target[2] - self._command[2]))
        with self._edit():
            for name, wheel in self._wheels.items():
                theta, omega = self._wheel_targets(name, *self._command)
                if theta is not None:
                    error = theta - self._steer_now[name]
                    step = PARAMS["STEER_RATE"] * dt
                    self._steer_now[name] += max(-step, min(step, error))
                    omega *= max(0.0, math.cos(math.radians(theta - self._steer_now[name])))
                wheel["steer_target"].Set(self._steer_now[name])
                wheel["drive_target"].Set(omega)
                self._drive_now[name] = omega
        return self.command

    def zero(self):
        """Remove velocity targets immediately; this is not proof of physical rest."""
        errors = []
        with self._edit():
            for name, wheel in self._wheels.items():
                try:
                    if wheel["drive_target"].Set(0.0) is False:
                        raise RuntimeError("USD rejected the zero target")
                    self._drive_now[name] = 0.0
                except Exception as exc:
                    errors.append(f"{name}: {exc}")
        self._command[:] = [0.0, 0.0, 0.0]
        if errors:
            raise RuntimeError("Could not clear RV1 wheel targets: " + "; ".join(errors))

    def reset_physics(self):
        try:
            self.zero()
        finally:
            self._tensor_body = self._tensor_sim = None

    @staticmethod
    def _row(data, length):
        data = data.numpy() if hasattr(data, "numpy") else data
        row = list(data[0])
        if len(row) != length:
            raise RuntimeError("Unexpected RV1 physics tensor shape")
        return [_number(float(v), "physics measurement") for v in row]

    def telemetry(self, playing):
        result = {"measurement_valid": False, "chassis_path": self.chassis_path,
                  "command": self.command, "geometry": self.geometry, "limits": self.limits,
                  "wheels": {name: {"position_body_m": list(w["pos"]),
                                    "steering_target_deg": self._steer_now[name],
                                    "drive_target_deg_s": self._drive_now[name]}
                             for name, w in self._wheels.items()}}
        if not playing:
            return result
        sim = self._simulation_manager.get_physics_simulation_view()
        if sim is None:
            raise RuntimeError("RV1 physics measurements are not initialized")
        if sim is not self._tensor_sim or self._tensor_body is None:
            body = sim.create_rigid_body_view(self.chassis_path)
            if body.count != 1:
                raise RuntimeError("RV1 telemetry requires exactly one chassis rigid body")
            self._tensor_sim, self._tensor_body = sim, body
        pose = self._row(self._tensor_body.get_transforms(), 7)
        velocity = self._row(self._tensor_body.get_velocities(), 6)
        norm = math.hypot(*pose[3:])
        if norm < 1e-8:
            raise RuntimeError("RV1 physics returned an invalid orientation")
        x, y, z, w = (v / norm for v in pose[3:])
        rotation = ((1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)),
                    (2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)),
                    (2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)))
        result.update(measurement_valid=True, position_m=pose[:3], orientation_xyzw=[x, y, z, w],
                      yaw_rad=math.atan2(rotation[1][0], rotation[0][0]),
                      roll_rad=math.atan2(rotation[2][1], rotation[2][2]),
                      pitch_rad=math.asin(max(-1.0, min(1.0, -rotation[2][0]))),
                      linear_velocity_world_mps=velocity[:3], angular_velocity_world_rps=velocity[3:],
                      linear_velocity_body_mps=[sum(rotation[i][j] * velocity[i] for i in range(3)) for j in range(3)],
                      speed_mps=math.hypot(*velocity[:3]), yaw_rate_rps=velocity[5])
        return result


class _Ros2Session:
    """Optional Script Editor lifecycle; all actuation remains in RV1Navigation."""

    def __init__(self, controller, navigation, app, timeline, usd_context, manager):
        self.controller, self.navigation = controller, navigation
        self._timeline, self._usd_context, self._manager = timeline, usd_context, manager
        self._subscription = None
        self._closed = False
        try:
            self.navigation.start()  # ROS resources exist; input is still disabled.
            self._subscription = app.get_update_event_stream().create_subscription_to_pop(
                self._update, name="rv1_nav2_cmd_vel")
        except Exception:
            self.navigation.close()
            raise

    def _update(self, _event):
        if self._closed:
            return
        try:
            if self._usd_context.get_stage() is not self.controller.stage:
                raise RuntimeError("RV1 stage changed; restart the ROS 2 session explicitly")
            self.navigation.update(float(self._manager.get_simulation_time()),
                                   playing=self._timeline.is_playing())
        except Exception:
            self.close()
            raise

    def arm(self):
        """Enable fresh Nav2 /cmd_vel after the caller starts simulation playback."""
        if self._closed or not self._timeline.is_playing():
            raise RuntimeError("Start simulation playback before arming the ROS 2 session")
        return self.navigation.arm()

    def status(self):
        return self.navigation.status()

    def close(self):
        if self._closed:
            return
        self._closed = True
        subscription, self._subscription = self._subscription, None
        try:
            if subscription is not None:
                subscription.unsubscribe()
        finally:
            try:
                self.navigation.close()
            finally:
                import builtins
                if getattr(builtins, "_rv1_nav2_standalone", None) is self:
                    del builtins._rv1_nav2_standalone


def start_ros2(runtime_path=None, config=None):
    """Explicit standalone /cmd_vel integration; importing/running does nothing.

    Requires Isaac Sim with its ROS 2 bridge enabled, a stopped RV1 stage, and
    this navigation plugin installed. ``runtime_path`` is the directory holding
    ``rv1/navigation.py`` (default: this controller's installed plugin root). The function
    returns a session with arm(), status() and close(). Start timeline playback
    separately, then call arm() when Nav2 is ready. Watchdog/pause/time resets
    disable input until a new explicit arm(). Do not run alongside the plugin's
    existing runtime adapter or a second RV1 /cmd_vel graph.
    """
    import builtins
    import importlib
    from pathlib import Path
    import sys

    if runtime_path is not None:
        root = Path(runtime_path).expanduser().resolve()
    else:
        root = Path(__file__).resolve().parents[1]
        if not (root / "rv1/navigation.py").is_file():
            import os
            root = (Path(os.environ.get("HERMES_HOME", str(Path.home() / ".hermes"))).expanduser()
                    / "plugins/isaac-sim").resolve()
    source = root / "rv1/navigation.py"
    if not source.is_file():
        raise ValueError("runtime_path must contain the installed rv1/navigation.py")
    existing = getattr(builtins, "_rv1_nav2_standalone", None)
    if existing is not None:
        raise RuntimeError("An RV1 standalone ROS 2 session already exists; close it first")
    # Keep the selected implementation importable for the lifetime of its ROS
    # session; refuse silently mixing different plugin copies in one process.
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    module = importlib.import_module("rv1.navigation")
    if Path(module.__file__).resolve() != source.resolve():
        raise RuntimeError("Another rv1.navigation module is already imported")

    import omni.kit.app
    import omni.timeline
    import omni.usd
    from isaacsim.core.simulation_manager import SimulationManager

    usd_context = omni.usd.get_context()
    controller = RV1Controller(usd_context.get_stage(), SimulationManager)
    navigation = module.RV1Navigation(controller, config)
    session = _Ros2Session(controller, navigation, omni.kit.app.get_app(),
                          omni.timeline.get_timeline_interface(), usd_context,
                          SimulationManager)
    builtins._rv1_nav2_standalone = session
    return session
