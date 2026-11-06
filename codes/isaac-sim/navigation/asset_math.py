"""Coordinate conversion and URDF adaptation without Isaac imports."""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections import Counter, deque
from array import array


def world_to_pixel(x, y, origin, resolution, width, height):
    """ROS lower-left origin -> PNG top-left row, +X right and +Y up."""
    col = math.floor((x - origin[0]) / resolution + 1e-7)
    row_from_bottom = math.floor((y - origin[1]) / resolution + 1e-7)
    if not 0 <= col < width or not 0 <= row_from_bottom < height:
        return None
    return height - 1 - row_from_bottom, col


def rasterize_positions(free_positions, occupied_positions, minimum, maximum, resolution, meters_per_unit=1.0):
    """Rasterize returned world positions, bypassing undocumented buffer order.

Occupied overrides free; cells not reached by the NVIDIA generator stay unknown.
No dilation, closing, synthetic free cells, or obstacle removal is performed.
"""
    origin = [float(minimum[0]) * meters_per_unit, float(minimum[1]) * meters_per_unit, 0.0]
    spans = [(float(maximum[i]) - float(minimum[i])) * meters_per_unit / resolution for i in (0, 1)]
    width, height = [int(round(s)) if math.isclose(s, round(s), abs_tol=1e-3) else math.ceil(s) for s in spans]
    if width <= 0 or height <= 0:
        raise ValueError("Generator returned an empty XY region")
    pixels = bytearray([127]) * (width * height)
    outside = 0
    for positions, value in ((free_positions, 255), (occupied_positions, 0)):
        for point in positions:
            index = world_to_pixel(float(point[0]) * meters_per_unit, float(point[1]) * meters_per_unit,
                                   origin, resolution, width, height)
            if index is None:
                outside += 1
            else:
                pixels[index[0] * width + index[1]] = value
    return pixels, width, height, origin, outside


def project_volume(free_positions, occupied_positions, minimum, maximum, resolution, meters_per_unit=1.0):
    """Collapse a native 3D map conservatively over its actual height range.

Any occupied voxel blocks its XY column. A column is free only if every Z cell
is explicitly free. Missing/unknown voxels never become traversable by union.
"""
    pixels, width, height, origin, outside = rasterize_positions(
        (), occupied_positions, minimum, maximum, resolution, meters_per_unit)
    zmin = float(minimum[2]) * meters_per_unit
    zspan = (float(maximum[2]) - float(minimum[2])) * meters_per_unit / resolution
    layers = max(1, int(round(zspan)) if math.isclose(zspan, round(zspan), abs_tol=1e-3) else math.ceil(zspan))
    # Deduplicate with bounded buffers instead of millions of Python sets.
    # Storage is volume_cells/8 + 4*XY_cells bytes, independent of duplication.
    seen = bytearray((width * height * layers + 7) // 8)
    free_counts = array("I", [0]) * (width * height)
    fractional_offsets = [set(), set(), set()]
    for positions in (free_positions, occupied_positions):
        for point in positions:
            for axis in range(3):
                units = (float(point[axis]) - float(minimum[axis])) * meters_per_unit / resolution
                fractional_offsets[axis].add(round(units - math.floor(units), 3))
    for point in free_positions:
        pixel = world_to_pixel(float(point[0]) * meters_per_unit, float(point[1]) * meters_per_unit,
                               origin, resolution, width, height)
        iz = math.floor((float(point[2]) * meters_per_unit - zmin) / resolution + 1e-6)
        if pixel is None or not 0 <= iz < layers:
            outside += 1
            continue
        index = pixel[0] * width + pixel[1]
        if pixels[index] != 0:
            voxel = index * layers + iz
            byte, bit = divmod(voxel, 8)
            mask = 1 << bit
            if not seen[byte] & mask:
                seen[byte] |= mask
                free_counts[index] += 1
    for index, count in enumerate(free_counts):
        if count == layers:
            pixels[index] = 255
    return pixels, width, height, origin, outside, {
        "height_cells": layers, "volume_cells": width * height * layers,
        "projection": "occupied_if_any;free_only_if_all_z_cells_free;otherwise_unknown",
        "voxel_fractional_offsets_from_min_bound": [sorted(values) for values in fractional_offsets],
        "raster_edges_m": [origin[0], origin[1], origin[0] + width * resolution, origin[1] + height * resolution],
        "raster_height_edges_m": [zmin, zmin + layers * resolution]}


def quality_metrics(pixels, width, height, seed_pixel=None):
    counts = Counter(pixels)
    total = width * height
    metrics = {"width": width, "height": height, "cells": total,
               "occupied_cells": counts[0], "free_cells": counts[255], "unknown_cells": counts[127],
               "occupied_fraction": counts[0] / total, "free_fraction": counts[255] / total,
               "unknown_fraction": counts[127] / total}
    seed_index = None if seed_pixel is None else seed_pixel[0] * width + seed_pixel[1]
    metrics["seed_is_free"] = seed_index is not None and pixels[seed_index] == 255
    connected = 0
    if metrics["seed_is_free"]:
        seen = bytearray(total)
        queue = deque([seed_index])
        seen[seed_index] = 1
        while queue:
            index = queue.popleft()
            connected += 1
            row, col = divmod(index, width)
            neighbors = []
            if col:
                neighbors.append(index - 1)
            if col + 1 < width:
                neighbors.append(index + 1)
            if row:
                neighbors.append(index - width)
            if row + 1 < height:
                neighbors.append(index + width)
            for neighbor in neighbors:
                if not seen[neighbor] and pixels[neighbor] == 255:
                    seen[neighbor] = 1
                    queue.append(neighbor)
    metrics["seed_connected_free_cells"] = connected
    metrics["usable_seed_region"] = connected > 0
    metrics["warnings"] = ([] if counts[0] else ["No occupied cells: inspect height band and collision geometry."])
    if not metrics["seed_is_free"]:
        metrics["warnings"].append("Seed is not free; this map is not ready for planning from the requested seed.")
    return metrics


def _validate_urdf_tree(root):
    """Reject incomplete/ambiguous topology instead of displaying a partial robot."""
    if root.tag != "robot":
        raise ValueError("URDF root element must be robot")
    links = [link.get("name") for link in root.findall("link")]
    joints = root.findall("joint")
    names = [joint.get("name") for joint in joints]
    if not links or any(not name for name in links + names):
        raise ValueError("URDF links and joints must have nonempty names")
    if len(set(links)) != len(links) or len(set(names)) != len(names):
        raise ValueError("URDF has duplicate link or joint names")
    parents = {}
    adjacency = {name: [] for name in links}
    for joint in joints:
        parent, child = joint.find("parent"), joint.find("child")
        if parent is None or child is None:
            raise ValueError(f"Joint {joint.get('name')} requires parent and child")
        source, target = parent.get("link"), child.get("link")
        if source not in adjacency or target not in adjacency:
            raise ValueError(f"Joint {joint.get('name')} references a missing link")
        if source == target or target in parents:
            raise ValueError(f"URDF has self-loop or multiple parents at {target}")
        parents[target] = source
        adjacency[source].append(target)
    bases = set(links) - parents.keys()
    if len(bases) != 1:
        raise ValueError(f"Expected one URDF root link, found {sorted(bases)}")
    base = next(iter(bases))
    visited, pending = set(), [base]
    while pending:
        name = pending.pop()
        if name in visited:
            raise ValueError("URDF contains a cycle")
        visited.add(name)
        pending.extend(adjacency[name])
    if visited != set(links):
        raise ValueError("URDF has disconnected or cyclic links")
    return base


def adapt_urdf_for_rviz(xml_text, base_frame="base_link", joint_efforts=None, robot_name=None):
    """Create a single-tree visualization URDF with a caller-chosen base/name.

NVIDIA's raw exporter may add world--floating_base. robot_state_publisher
cannot provide that dynamic transform. Remove only that explicit generated
world root, preserving every physical link/joint and its geometry.
"""
    root = ET.fromstring(xml_text)
    _validate_urdf_tree(root)
    if root.find("loop_joint") is not None:
        raise ValueError("Closed-chain loop joints are not supported by this visualization exporter")
    source_robot_name = root.get("name")
    if robot_name is not None:
        root.set("name", robot_name)
    changes = []
    for joint in list(root.findall("joint")):
        parent = joint.find("parent")
        if joint.get("type") == "floating" and parent is not None and parent.get("link") == "world":
            world = next((link for link in root.findall("link") if link.get("name") == "world"), None)
            world_edges = [item for item in root.findall("joint")
                           if any(item.find(tag).get("link") == "world" for tag in ("parent", "child"))]
            if joint.get("name") != "floating_base" or world is None or len(world) or len(world_edges) != 1:
                raise ValueError("Refusing to remove a physical or ambiguous world link")
            root.remove(joint)
            root.remove(world)
            changes.append({"removed_floating_joint": joint.get("name"), "removed_link": "world"})
    links = [x.get("name") for x in root.findall("link")]
    original_base = _validate_urdf_tree(root)
    if original_base != base_frame:
        if base_frame in links:
            raise ValueError(f"Cannot rename root to {base_frame}: link already exists")
        for link in root.findall("link"):
            if link.get("name") == original_base:
                link.set("name", base_frame)
        for joint in root.findall("joint"):
            for tag in ("parent", "child"):
                endpoint = joint.find(tag)
                if endpoint.get("link") == original_base:
                    endpoint.set("link", base_frame)
        changes.append({"renamed_root_link": original_base, "to": base_frame})
    links = [x.get("name") for x in root.findall("link")]
    joints = [x.get("name") for x in root.findall("joint")]
    if any(x.get("type") not in ("fixed", "continuous", "revolute", "prismatic") for x in root.findall("joint")):
        raise ValueError("Unsupported joint type in RViz description; only fixed/continuous/revolute/prismatic are supported")
    # urdfdom rejects NVIDIA's omitted effort attributes and literal inf values.
    # This description drives visualization, never Isaac physics or actuators.
    limit_changes = []
    if joint_efforts is not None:
        for joint in root.findall("joint"):
            if joint.get("type") == "fixed":
                continue
            name = joint.get("name")
            limit = joint.find("limit")
            change = {"joint": name}
            if limit is None:
                if joint.get("type") != "continuous":
                    raise ValueError(f"Exporter omitted limits for movable joint {name}")
                # The native converter skips _read_limits for continuous
                # joints. Their effort still comes from the selected USD
                # drive; velocity follows the documented display placeholder.
                limit = ET.SubElement(joint, "limit")
                change["added_continuous_joint_limit"] = True
            lower, upper = limit.get("lower"), limit.get("upper")
            if joint.get("type") == "revolute" and lower is not None and upper is not None:
                if float(lower) == -math.inf and float(upper) == math.inf:
                    joint.set("type", "continuous")
                    del limit.attrib["lower"]
                    del limit.attrib["upper"]
                    change["unbounded_revolute_to_continuous"] = True
                elif not math.isfinite(float(lower)) or not math.isfinite(float(upper)):
                    raise ValueError(f"Joint {name} has partially unbounded revolute limits")
            if "effort" not in limit.attrib or not math.isfinite(float(limit.get("effort"))):
                effort = joint_efforts.get(name)
                if effort is None or not math.isfinite(effort) or effort < 0:
                    raise ValueError(f"Joint {name} has no finite USD drive maxForce for URDF effort")
                limit.set("effort", format(effort, ".9g"))
                change["effort_from_usd_drive_maxForce"] = effort
            if "velocity" not in limit.attrib or not math.isfinite(float(limit.get("velocity"))):
                # Passive joints may lack a finite velocity limit. Zero is an
                # explicit non-actuated visualization placeholder,
                # not an invented physical bound. JointState display ignores it.
                limit.set("velocity", "0")
                change["velocity_placeholder"] = {"value": 0, "scope": "visualization_only",
                                                   "reason": "USD has no finite velocity limit"}
            if len(change) > 1:
                limit_changes.append(change)
            if float(limit.get("effort")) < 0 or float(limit.get("velocity")) < 0:
                raise ValueError(f"Joint {name} has a negative effort or velocity limit")
            if joint.get("type") in ("revolute", "prismatic"):
                bounds = [limit.get("lower"), limit.get("upper")]
                if any(value is None or not math.isfinite(float(value)) for value in bounds):
                    raise ValueError(f"Joint {name} requires finite lower and upper limits")
                if float(bounds[0]) > float(bounds[1]):
                    raise ValueError(f"Joint {name} has reversed position limits")
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=True), {
        "source_base_link": original_base, "base_frame": base_frame, "changes": changes,
        "robot_name": root.get("name"), "source_robot_name": source_robot_name,
        "topology": "single_connected_tree",
        "limit_normalization": limit_changes, "description_use": "visualization_only_not_actuator_limits",
        "links": links, "joints": joints,
        "movable_joints": [x.get("name") for x in root.findall("joint") if x.get("type") != "fixed"]}
