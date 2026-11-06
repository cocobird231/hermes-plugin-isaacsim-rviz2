#!/usr/bin/env python3
"""Isolated Isaac Sim asset conversion process.

Run with the host's Isaac Python, never inside the interactive simulator:
    python asset_worker.py --request request.json --result result.json
No source layer is saved. All temporary USD edits target the session layer.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parent))
from assets import resolve_map_bounds, validate_asset_request
from asset_math import adapt_urdf_for_rviz, project_volume, quality_metrics, world_to_pixel


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path, value):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temp = target.with_name(target.name + ".partial")
    with temp.open("w") as stream:
        stream.write(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp, target)
    descriptor = os.open(target.parent, os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _extension_versions(manager, names):
    result = {}
    for name in names:
        extension_id = manager.get_enabled_extension_id(name)
        result[name] = extension_id
    return result


def _load_stage(app, request):
    import omni.usd
    from pxr import Usd
    context = omni.usd.get_context()
    if not context.open_stage(request["source_path"]):
        raise RuntimeError("Isaac could not open source stage")
    deadline = time.monotonic() + 240
    for _ in range(3):
        app.update()
    while context.get_stage_loading_status()[2] > 0:
        if time.monotonic() > deadline:
            raise TimeoutError("Timed out waiting for USD references and payloads")
        app.update()
    stage = context.get_stage()
    if stage is None:
        raise RuntimeError("Source stage did not load")
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    return context, stage


def _scene_geometry_bounds(stage, meters_per_unit):
    """All enabled collision Gprims, after requested exclusions/conversion.

    Native omap queries PhysX, so unmeshed Gaussian/volume render bounds cannot
    contribute occupancy geometry. Record those separately, never pretend that
    they were mapped. Invisible colliders still count because PhysX sees them.
    This is XY extent discovery, not a ground-support or slope assessment.
    """
    from pxr import Usd, UsdGeom, UsdPhysics
    purposes = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy]
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), purposes + [UsdGeom.Tokens.guide],
                             useExtentsHint=False, ignoreVisibility=True)
    lower, upper, count = [math.inf] * 3, [-math.inf] * 3, 0
    geometries = []
    excluded = []
    visual_lower, visual_upper = [math.inf] * 3, [-math.inf] * 3
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Gprim):
            continue
        imageable = UsdGeom.Imageable(prim)
        visible = imageable.ComputeVisibility() != UsdGeom.Tokens.invisible and imageable.ComputePurpose() in purposes
        collider = (prim.HasAPI(UsdPhysics.CollisionAPI)
                    and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not False)
        if not visible and not collider:
            continue
        bounds = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if bounds.IsEmpty():
            continue
        minimum, maximum = list(bounds.GetMin()), list(bounds.GetMax())
        if not all(math.isfinite(x) for x in minimum + maximum):
            raise ValueError(f"Geometry has non-finite bounds: {prim.GetPath()}")
        record = {"prim_path": str(prim.GetPath()), "type": prim.GetTypeName(),
                  "minimum_m": [x * meters_per_unit for x in minimum],
                  "maximum_m": [x * meters_per_unit for x in maximum]}
        if visible:
            for axis in range(3):
                visual_lower[axis] = min(visual_lower[axis], record["minimum_m"][axis])
                visual_upper[axis] = max(visual_upper[axis], record["maximum_m"][axis])
        if not collider:
            excluded.append({**record, "reason": "not_enabled_collision_geometry_native_omap_cannot_map_it"})
            continue
        for axis in range(3):
            lower[axis] = min(lower[axis], minimum[axis] * meters_per_unit)
            upper[axis] = max(upper[axis], maximum[axis] * meters_per_unit)
        count += 1
        geometries.append(record)
    if not count:
        raise ValueError("No enabled bounded collision geometry for full_scene map extent discovery")
    return {"minimum_m": lower, "maximum_m": upper, "geometry_prim_count": count,
            "method": "union_enabled_collision_Gprim_world_bounds_after_exclusions",
            "ground_support_validated": False, "geometries": geometries,
            "unmapped_visual_geometry": excluded,
            "all_visible_geometry_bounds_m": {"minimum_m": visual_lower, "maximum_m": visual_upper}
                if all(math.isfinite(x) for x in visual_lower + visual_upper) else None}


def export_map(app, request, manager):
    import omni.physx
    import omni.timeline
    from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics
    from isaacsim.asset.gen.omap.bindings import _omap
    from PIL import Image

    context, stage = _load_stage(app, request)
    meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
    if not math.isfinite(meters_per_unit) or meters_per_unit <= 0:
        raise ValueError("Stage meters_per_unit must be finite and positive")
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
        raise ValueError("Map conversion requires a Z-up stage; no implicit axis rotation is applied")
    edits = {"disabled_graphs": [], "disabled_rigid_bodies": [], "excluded_prims": [], "added_colliders": 0}
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for path in request["exclude_prim_paths"]:
            prim = stage.GetPrimAtPath(path)
            if not prim:
                raise ValueError(f"Excluded prim does not exist: {path}")
            prim.SetActive(False)
            edits["excluded_prims"].append(path)
        for prim in list(stage.Traverse()):
            # A conversion worker must not publish ROS messages or run source controllers.
            if "OmniGraph" in prim.GetTypeName():
                prim.SetActive(False)
                edits["disabled_graphs"].append(str(prim.GetPath()))
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
                edits["disabled_rigid_bodies"].append(str(prim.GetPath()))
            if request["geometry_mode"] == "visual_meshes" and prim.IsA(UsdGeom.Mesh):
                if UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible:
                    continue
                points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
                if points is None or not len(points):
                    continue
                UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")
                edits["added_colliders"] += 1
        scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
        if len(scenes) > 1:
            raise ValueError("Multiple physics scenes are not supported by the export worker")
        scene = UsdPhysics.Scene(scenes[0]) if scenes else UsdPhysics.Scene.Define(stage, "/NavigationExportPhysics")
        api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        api.CreateEnableGPUDynamicsAttr(False)
        api.CreateBroadphaseTypeAttr("MBP")
    collider_count = sum(1 for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.CollisionAPI)
                         and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not False)
    if not collider_count:
        raise ValueError("No enabled collision geometry; request geometry_mode=visual_meshes for mesh assets")
    scene_bounds = _scene_geometry_bounds(stage, meters_per_unit) if request["bounds_mode"] == "full_scene" else None
    print("ASSET_SCENE_GEOMETRY " + json.dumps(scene_bounds), flush=True)
    resolved_bounds, budget = resolve_map_bounds(
        request, scene_bounds["minimum_m"] if scene_bounds else None,
        scene_bounds["maximum_m"] if scene_bounds else None)
    print("ASSET_MAP_BOUNDS " + json.dumps({"bounds_mode": request["bounds_mode"],
          "resolved_bounds_stage_xy_m": resolved_bounds, "scene_geometry_bounds": scene_bounds,
          "allocation_budget": budget}), flush=True)
    timeline = omni.timeline.get_timeline_interface()
    for _ in range(3):
        app.update()
    timeline.play()
    for _ in range(3):
        app.update()
    generator = _omap.Generator(omni.physx.get_physx_interface(), context.get_stage_id())
    resolution_units = request["resolution_m"] / meters_per_unit
    generator.update_settings(resolution_units, 100, 0, -1)
    seed = [x / meters_per_unit for x in request["seed_stage_xyz_m"]]
    xmin, ymin, xmax, ymax = resolved_bounds
    desired_min = [xmin / meters_per_unit, ymin / meters_per_unit, request["z_min_m"] / meters_per_unit]
    desired_max = [xmax / meters_per_unit, ymax / meters_per_unit, request["z_max_m"] / meters_per_unit]
    # omap 2.3.2 excludes the last cell at its supplied upper bound. Verified
    # positions lie half a cell from returned minimum bounds; extend the API
    # upper bound by one cell so the actual output covers the requested region.
    native_max = [value + resolution_units for value in desired_max]
    # The older OccupancyMap interface documents relative bounds; Generator's
    # current binding documents absolute bounds. Detect the installed behavior
    # from its returned bounds instead of hard-coding a seed-dependent offset.
    attempts = []
    for convention in ("relative_to_seed", "absolute"):
        lower = desired_min if convention == "absolute" else [desired_min[i] - seed[i] for i in range(3)]
        upper = native_max if convention == "absolute" else [native_max[i] - seed[i] for i in range(3)]
        generator.set_transform(tuple(seed), tuple(lower), tuple(upper))
        # generate2d() in omap 2.3.2 samples only seed Z despite a Z range.
        # generate3d() followed by conservative column projection honors height.
        generator.generate3d()
        app.update()
        minimum, maximum = list(generator.get_min_bound()), list(generator.get_max_bound())
        attempts.append({"convention": convention, "minimum_stage_units": minimum, "maximum_stage_units": maximum})
        tolerance = resolution_units * 1.1
        if all(abs(minimum[i] - desired_min[i]) <= tolerance and abs(maximum[i] - desired_max[i]) <= tolerance for i in (0, 1, 2)):
            break
    else:
        raise RuntimeError(f"Generator bounds do not match requested world ROI: {attempts}")
    coverage_tolerance = resolution_units * .02
    if not all(minimum[i] <= desired_min[i] + coverage_tolerance and maximum[i] >= desired_max[i] - coverage_tolerance for i in (0, 1, 2)):
        raise RuntimeError(f"Generator volume does not cover requested XYZ bounds: {attempts}")
    free_positions, occupied_positions = generator.get_free_positions(), generator.get_occupied_positions()
    pixels, width, height, origin, outside, projection = project_volume(
        free_positions, occupied_positions, minimum, maximum, request["resolution_m"], meters_per_unit)
    if width * height > request["max_cells"]:
        raise ValueError("Actual map exceeds max_cells")
    if projection["volume_cells"] > request["max_voxels"]:
        raise ValueError("Actual map volume exceeds max_voxels")
    seed_pixel = world_to_pixel(*request["seed_stage_xyz_m"][:2], origin, request["resolution_m"], width, height)
    quality = quality_metrics(pixels, width, height, seed_pixel)
    quality.update(returned_free_positions=len(free_positions), returned_occupied_positions=len(occupied_positions),
                   positions_outside_image=outside, enabled_colliders=collider_count)
    quality["ground_support_validated"] = False
    quality["warnings"].append(
        "Fixed world-height occupancy does not validate ground support, slope, or every road height; "
        "free cells require terrain/footprint verification before use as navigation goals.")
    if scene_bounds and scene_bounds["unmapped_visual_geometry"]:
        quality["warnings"].append(
            "Visual geometry without enabled PhysX colliders is not mapped; inspect "
            "scene_geometry_bounds.unmapped_visual_geometry (for example Gaussian splat volumes).")
    timeline.stop()
    out = Path(request["output_dir"])
    image_path, yaml_path = out / "map.png", out / "map.yaml"
    Image.frombytes("L", (width, height), bytes(pixels)).save(image_path)
    yaml_path.write_text(
        f"image: map.png\nmode: trinary\nresolution: {request['resolution_m']}\n"
        f"origin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\n"
        f"occupied_thresh: {request['occupied_thresh']}\nfree_thresh: {request['free_thresh']}\n")
    return {"outputs": {"image": str(image_path), "map_yaml": str(yaml_path)}, "quality": quality,
            "metadata": {"meters_per_unit": meters_per_unit, "up_axis": "Z", "image_origin": "top_left",
                         "image_axis": {"columns": "+world_x", "rows": "-world_y"},
                         "map_origin_m": origin, "actual_min_bound_m": [x * meters_per_unit for x in minimum],
                         "actual_max_bound_m": [x * meters_per_unit for x in maximum],
                         "bounds_mode": request["bounds_mode"], "resolved_bounds_stage_xy_m": resolved_bounds,
                         "scene_geometry_bounds": scene_bounds, "allocation_budget": budget,
                         "whole_scene_xy_covered": scene_bounds is not None,
                         "whole_scene_xy_scope": "enabled_collision_Gprim_geometry_not_unmeshed_visual_volumes",
                         "T_map_from_stage_m": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
                         "generator": "isaacsim.asset.gen.omap.bindings._omap.Generator.generate3d",
                         "bounds_attempts": attempts, "generator_dimensions": list(generator.get_dimensions()),
                         "native_upper_bound_padding_cells": 1, "requested_xyz_bounds_covered": True,
                         "height_mode": "native_generate3d_conservative_column_projection", "projection": projection,
                         "native_dimensions_note": "Native integer dimensions can truncate float bounds; raster dimensions derive from actual world bounds.",
                         "session_edits": edits,
                         "extension_versions": _extension_versions(manager, ["isaacsim.asset.gen.omap"]),
                         "source_files_saved": False}}


def select_robot_subtree(stage, root_prim_path=None):
    """Resolve an unambiguous articulation before NVIDIA's permissive discovery.

    The native exporter can fall back outside an explicitly requested root and
    omit disconnected bodies. Neither behavior is acceptable for an agent's
    generic robot export. Selection and validation do not save a USD layer.
    """
    from pxr import Usd, UsdPhysics

    def subtree(prim):
        return list(Usd.PrimRange(prim, Usd.TraverseInstanceProxies()))

    if root_prim_path:
        root = stage.GetPrimAtPath(root_prim_path)
        if not root or root.IsPseudoRoot():
            raise ValueError("Requested root_prim_path must exist and cannot be the stage pseudo-root")
    else:
        articulations = [prim for prim in subtree(stage.GetPseudoRoot())
                         if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
        if len(articulations) != 1:
            raise ValueError(f"Expected exactly one articulation; found {len(articulations)}. Specify root_prim_path for a robot subtree")
        root = stage.GetDefaultPrim()
        if not root or not articulations[0].GetPath().HasPrefix(root.GetPath()):
            # Use the containing top-level subtree so sibling joints are not
            # lost when ArticulationRootAPI is applied to a rigid body.
            root = articulations[0]
            while not root.GetParent().IsPseudoRoot():
                root = root.GetParent()
    prims = subtree(root)
    articulations = [prim for prim in prims if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(articulations) != 1:
        raise ValueError(f"Requested robot subtree must contain exactly one articulation; found {len(articulations)} (stage-wide fallback is disabled)")
    rigid = {str(prim.GetPath()) for prim in prims if prim.HasAPI(UsdPhysics.RigidBodyAPI)}
    if not rigid:
        raise ValueError("Selected robot has no rigid body links")
    joints = [prim for prim in prims if prim.IsA(UsdPhysics.Joint)]
    adjacency = {path: set() for path in rigid}
    edge_count = 0
    for prim in joints:
        if not any(prim.IsA(kind) for kind in (UsdPhysics.FixedJoint, UsdPhysics.RevoluteJoint, UsdPhysics.PrismaticJoint)):
            raise ValueError(f"Unsupported USD joint type {prim.GetTypeName()} at {prim.GetPath()}; only fixed/revolute/prismatic are supported")
        joint = UsdPhysics.Joint(prim)
        if joint.GetExcludeFromArticulationAttr().Get():
            raise ValueError(f"Closed-chain/excluded articulation joint is not supported: {prim.GetPath()}")
        if joint.GetJointEnabledAttr().Get() is False:
            raise ValueError(f"Disabled joint would change exported topology: {prim.GetPath()}")
        endpoints = []
        for relationship in (joint.GetBody0Rel(), joint.GetBody1Rel()):
            targets = relationship.GetTargets()
            if len(targets) > 1:
                raise ValueError(f"Joint has multiple body targets: {prim.GetPath()}")
            if not targets:
                continue
            path = targets[0]
            if not path.HasPrefix(root.GetPath()):
                raise ValueError(f"Joint references a body outside root_prim_path: {path}")
            body = stage.GetPrimAtPath(path)
            if not body:
                raise ValueError(f"Joint references a missing USD body: {path}")
            if str(path) in rigid:
                endpoints.append(str(path))
        if len(endpoints) == 2:
            left, right = endpoints
            if left == right or right in adjacency[left]:
                raise ValueError(f"Self-loop or parallel joint in robot topology: {prim.GetPath()}")
            adjacency[left].add(right)
            adjacency[right].add(left)
            edge_count += 1
        elif not prim.IsA(UsdPhysics.FixedJoint) or len(endpoints) != 1:
            raise ValueError(f"Only a fixed world-to-base joint may have fewer than two rigid bodies: {prim.GetPath()}")
    visited, pending = set(), [next(iter(rigid))]
    while pending:
        path = pending.pop()
        if path in visited:
            continue
        visited.add(path)
        pending.extend(adjacency[path] - visited)
    if visited != rigid or edge_count != len(rigid) - 1:
        raise ValueError("Robot rigid bodies must form one connected acyclic joint tree")
    return root, joints, {"root_prim_path": str(root.GetPath()),
                          "articulation_prim_path": str(articulations[0].GetPath()),
                          "rigid_body_paths": sorted(rigid),
                          "joint_prim_paths": [str(prim.GetPath()) for prim in joints],
                          "selection": "explicit_subtree" if root_prim_path else "single_articulation_subtree"}


def export_urdf(app, request, manager):
    from isaacsim.asset.exporter.urdf import UsdToUrdfConverter
    from isaacsim.asset.exporter.urdf.converter.robot_finder import find_robot
    from isaacsim.asset.exporter.urdf.converter.transform_utils import get_prim_name
    from pxr import UsdGeom, UsdPhysics
    _, stage = _load_stage(app, request)
    root, selected_joints, selection = select_robot_subtree(stage, request.get("root_prim_path"))
    meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
    if not math.isfinite(meters_per_unit) or meters_per_unit <= 0:
        raise ValueError("Stage meters_per_unit must be finite and positive")
    # The installed converter uses stage transforms directly and does not
    # rescale lengths to SI. Reject other units rather than export a wrong size.
    if not math.isclose(meters_per_unit, 1.0, rel_tol=0, abs_tol=1e-12):
        raise ValueError("URDF conversion currently requires meters_per_unit=1; implicit unit conversion is not supported")
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
        raise ValueError("URDF conversion currently requires Z-up; implicit axis conversion is not supported")
    description = find_robot(stage, str(root.GetPath()))
    if {str(prim.GetPath()) for prim in description.ordered_links} != set(selection["rigid_body_paths"]):
        raise ValueError("NVIDIA exporter discovery omitted selected robot bodies")
    if description.loop_joints:
        raise ValueError("Closed-chain robot export is unsupported")
    out = Path(request["output_dir"])
    robot_name = request.get("robot_name", "robot")
    raw_path, rviz_path = out / f"{robot_name}.raw.urdf", out / f"{robot_name}.urdf"
    converter = UsdToUrdfConverter(stage, root_prim_path=str(root.GetPath()),
                                  mesh_dir_name="meshes", mesh_path_prefix="./")
    converter.convert(str(raw_path))
    joint_efforts = {}
    for prim in selected_joints:
        name = get_prim_name(prim)
        drive = "linear" if prim.IsA(UsdPhysics.PrismaticJoint) else "angular"
        force = prim.GetAttribute(f"drive:{drive}:physics:maxForce").Get()
        if force is not None:
            if name in joint_efforts:
                raise ValueError(f"Ambiguous duplicate USD joint name {name}")
            joint_efforts[name] = float(force)
    adapted, details = adapt_urdf_for_rviz(raw_path.read_text(), request["base_frame"], joint_efforts, robot_name)
    robot = ET.fromstring(adapted)
    missing = []
    for mesh in robot.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith("file://"):
            target = Path(filename[7:]).resolve()
        elif "://" in filename:
            raise ValueError(f"Unsupported exported mesh URI: {filename}")
        else:
            target = (out / filename).resolve()
        if not target.is_relative_to(out):
            raise ValueError("Exported mesh reference points outside this output directory")
        if not target.is_file():
            missing.append(str(target))
        # RViz's resource retriever requires a URI; raw URDF remains portable.
        mesh.set("filename", target.as_uri())
    if missing:
        raise RuntimeError(f"Exported URDF has missing meshes: {missing}")
    ET.indent(robot, space="  ")
    rviz_path.write_text(ET.tostring(robot, encoding="unicode", xml_declaration=True) + "\n")
    meshes = sorted(str(path) for path in (out / "meshes").rglob("*") if path.is_file())
    return {"outputs": {"urdf": str(rviz_path), "raw_urdf": str(raw_path), "mesh_dir": str(out / "meshes")},
            "quality": {"valid_xml": True, "link_count": len(details["links"]), "joint_count": len(details["joints"]),
                        "mesh_file_count": len(meshes), "missing_meshes": []},
            "metadata": {**details, "meters_per_unit": meters_per_unit, "robot_selection": selection,
                         "fixed_base_in_source": description.is_fixed_base,
                         "mesh_files": meshes, "source_files_saved": False,
                         "extension_versions": _extension_versions(manager, ["isaacsim.asset.exporter.urdf"]),
                         "mesh_path_note": f"{raw_path.name} uses relative meshes; {rviz_path.name} uses host absolute file URIs"}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request", required=True)
    parser.add_argument("--result", required=True)
    args = parser.parse_args()
    started = time.time()
    app = None
    source = None
    result = {"ok": False, "started_unix": started, "worker_pid": os.getpid(),
              "shutdown_strategy": "isolated_worker_process_exit", "kit_graceful_cleanup": False,
              "shutdown_note": "Isaac 6.0.1 teardown faults remain unresolved; this isolated process exits after durable result writes."}
    try:
        request = validate_asset_request(json.loads(Path(args.request).read_text()))
        source = Path(request["source_path"])
        before_hash = sha256(source)
        result.update(operation=request["operation"], request=request, source_sha256=before_hash)
        output = Path(request["output_dir"])
        # A caller can pre-create an empty output directory, but cannot overwrite an export.
        if output.exists() and any(output.iterdir()):
            raise FileExistsError("output_dir must be new or empty")
        output.mkdir(parents=True, exist_ok=True)
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "hide_ui": True, "create_new_stage": False,
                             "disable_viewport_updates": True, "multi_gpu": False,
                             # Not relied on for lifecycle: this isolated worker
                             # terminates after durable writes, below.
                             "fast_shutdown": False})
        import omni.kit.app
        manager = omni.kit.app.get_app().get_extension_manager()
        extension = "isaacsim.asset.gen.omap" if request["operation"] == "map" else "isaacsim.asset.exporter.urdf"
        if not manager.set_extension_enabled_immediate(extension, True):
            raise RuntimeError(f"Could not enable extension {extension}")
        for _ in range(3):
            app.update()
        payload = export_map(app, request, manager) if request["operation"] == "map" else export_urdf(app, request, manager)
        result.update(payload)
        result["source_unchanged"] = sha256(source) == before_hash
        if not result["source_unchanged"]:
            raise RuntimeError("Source file changed during conversion")
        result["ok"] = True
        result["elapsed_seconds"] = time.time() - started
        metadata = output / "metadata.json"
        result["outputs"]["metadata"] = str(metadata)
        result["output_sha256"] = {str(path.relative_to(output)): sha256(path) for path in output.rglob("*") if path.is_file()}
        for path in output.rglob("*"):
            if path.is_file():
                with path.open("rb") as stream:
                    os.fsync(stream.fileno())
        write_json(metadata, result)
    except Exception as exc:
        result.update(error=f"{type(exc).__name__}: {exc}", traceback=traceback.format_exc(), elapsed_seconds=time.time() - started)
        if source and source.is_file() and "source_sha256" in result:
            result["source_unchanged"] = sha256(source) == result["source_sha256"]
    finally:
        write_json(args.result, result)
        print("ASSET_EXPORT_RESULT " + json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
        if app is not None:
            # This is a one-shot process, never the user's interactive Isaac.
            # Both Kit fast shutdown (busy TaskGroup abort) and full extension
            # teardown (segfault) fail on the tested 6.0.1 installation. Do not
            # mask conversion failures: preserve the validated result as status.
            # OS process termination reclaims this worker's threads/GPU context.
            sys.stdout.flush()
            sys.stderr.flush()
            for stream in (sys.stdout, sys.stderr):
                try:
                    os.fsync(stream.fileno())
                except OSError:
                    pass  # Pipes/terminals are flushed but cannot be fsynced.
            os._exit(0 if result["ok"] else 1)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
