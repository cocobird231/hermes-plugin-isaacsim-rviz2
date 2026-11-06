"""Reviewed Gongguan composition. Only authors the transient USD session layer."""
from pathlib import Path

from deployment import asset_path
MESH_PATH = "/World/gauss/mesh"
ROOT = "/World/AgentExperiment"
# Selected from offline road-mesh analysis; final values accompany experiment evidence.
SPAWN = (160.0, -155.0, 1.05)
YAW_DEGREES = 0.0


def validate_target(stage):
    from pxr import UsdGeom, UsdPhysics
    if not stage:
        raise ValueError("Load Gongguan before inserting RV1")
    actual = stage.GetRootLayer().realPath.split("[", 1)[0]
    if str(Path(actual).resolve()) != asset_path("gongguan_usd"):
        raise ValueError("RV1 insertion requires the registered Gongguan USDZ stage")
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z or abs(UsdGeom.GetStageMetersPerUnit(stage) - 1) > 1e-6:
        raise ValueError("Expected Z-up map with one meter per unit")
    if stage.GetPrimAtPath(ROOT).IsValid():
        raise ValueError("AgentExperiment already exists; stop and reload Gongguan to reset, do not insert twice")
    mesh = stage.GetPrimAtPath(MESH_PATH)
    if not mesh or not mesh.IsA(UsdGeom.Mesh) or not mesh.HasAPI(UsdPhysics.CollisionAPI):
        raise ValueError("Expected original Gongguan mesh collider")
    if not UsdPhysics.CollisionAPI(mesh).GetCollisionEnabledAttr().Get():
        raise ValueError("Original road collision is disabled")
    if UsdPhysics.MeshCollisionAPI(mesh).GetApproximationAttr().Get() != "none":
        raise ValueError("Expected original static triangle-mesh collision")
    if not Path(asset_path("rv1_usd")).is_file():
        raise ValueError("Original RV1 USD is missing")


def insert_rv1(stage):
    from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, PhysxSchema
    validate_target(stage)
    map_path, robot_path = asset_path("gongguan_usd"), asset_path("rv1_usd")
    session = stage.GetSessionLayer()
    before = session.ExportToString()
    try:
        with Usd.EditContext(stage, session):
            UsdGeom.Xform.Define(stage, ROOT)
            robot = UsdGeom.Xform.Define(stage, ROOT + "/RV1")
            if not robot.GetPrim().GetReferences().AddReference(robot_path):
                raise RuntimeError("Could not reference RV1 asset")
            robot.AddTranslateOp().Set(Gf.Vec3d(*SPAWN))
            robot.AddRotateZOp().Set(YAW_DEGREES)
            graph = stage.GetPrimAtPath(ROOT + "/RV1/rv1/ActionGraph")
            if graph:
                graph.SetActive(False)
            physics_scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
            if len(physics_scenes) > 1:
                raise ValueError("Multiple physics scenes are not supported in this experiment")
            physics = (UsdPhysics.Scene(physics_scenes[0]) if physics_scenes
                       else UsdPhysics.Scene.Define(stage, ROOT + "/PhysicsScene"))
            physics.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
            physics.CreateGravityMagnitudeAttr().Set(9.81)
            physx = PhysxSchema.PhysxSceneAPI.Apply(physics.GetPrim())
            physx.CreateTimeStepsPerSecondAttr().Set(960)
            physx.CreateSolverTypeAttr().Set("TGS")
            # Map animation is authored for a static capture. Avoid short loops.
            stage.SetStartTimeCode(0)
            stage.SetEndTimeCode(100000)
            stage.SetTimeCodesPerSecond(60)
            light = UsdLux.DomeLight.Define(stage, ROOT + "/FillLight")
            light.CreateIntensityAttr().Set(1000)
            camera = UsdGeom.Camera.Define(stage, ROOT + "/OverviewCamera")
            x, y, z = SPAWN
            # Stay over the roadway; the initial high roadside view intersected
            # reconstructed tree foliage and obscured the front of the vehicle.
            # Runtime follows the measured chassis translation using these
            # same offsets. Fixed world orientation keeps turns visible.
            eye_offset, target_offset = (10.0, -3.0, 3.45), (1.0, 0.0, -0.05)
            eye = Gf.Vec3d(x + eye_offset[0], y + eye_offset[1], z + eye_offset[2])
            target = Gf.Vec3d(x + target_offset[0], y + target_offset[1], z + target_offset[2])
            camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse())
            camera.CreateFocalLengthAttr().Set(24)
            camera.CreateHorizontalApertureAttr().Set(20.955)
            camera.CreateVerticalApertureAttr().Set(15.2908)
            camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.1, 10000))
            chassis = stage.GetPrimAtPath(ROOT + "/RV1/rv1/base_link/chassis_link")
            if not chassis or not chassis.HasAPI(UsdPhysics.RigidBodyAPI):
                raise RuntimeError("RV1 reference did not compose its chassis")
        return {"map_path": map_path, "robot_source": robot_path, "robot_prim": str(robot.GetPath()),
                "collision_mesh": MESH_PATH, "collision_mode": "original_static_triangle_mesh",
                "spawn_position_m": list(SPAWN), "spawn_yaw_degrees": YAW_DEGREES,
                "physics_scene": str(physics.GetPath()), "physics_hz": 960, "solver": "TGS",
                "camera_path": str(camera.GetPath()), "camera_eye_m": list(eye), "camera_target_m": list(target),
                "camera_follow": {"mode": "physics_translation", "position_source": "physx_tensor",
                                  "eye_offset_m": list(eye_offset), "target_offset_m": list(target_offset),
                                  "status": "waiting_for_physics", "update_count": 0},
                "edit_layer": "session", "source_files_saved": False}
    except Exception:
        session.ImportFromString(before)
        raise
