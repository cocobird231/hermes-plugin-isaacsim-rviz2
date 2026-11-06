#!/usr/bin/env python3
"""Read-only USD vs URDF neutral-pose visual bounds audit; no Kit or GPU."""
import argparse
import json
import math
from pathlib import Path
from urllib.parse import unquote, urlsplit
from portable_common import private_output
import xml.etree.ElementTree as ET

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics


def origin(element):
    transform = np.eye(4)
    if element is None:
        return transform
    transform[:3, 3] = [float(x) for x in element.get("xyz", "0 0 0").split()]
    r, p, y = [float(x) for x in element.get("rpy", "0 0 0").split()]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    transform[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                         [-sp, cp * sr, cp * cr]]
    return transform


def transformed(points, matrix):
    values = np.c_[np.array(points, dtype=float), np.ones(len(points))]
    return (matrix @ values.T).T[:, :3]


def bounds(points):
    points = np.concatenate(points)
    return np.array([points.min(axis=0), points.max(axis=0)])


def audit(source, urdf):
    stage = Usd.Stage.Open(str(source))
    robot = ET.parse(urdf).getroot()
    bodies = {p.GetName(): p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    chassis_inverse = UsdGeom.Xformable(bodies["chassis_link"]).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse()
    transforms = {"base_link": np.eye(4)}
    pending = list(robot.findall("joint"))
    while pending:
        unresolved = []
        for joint in pending:
            parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
            if parent in transforms:
                transforms[child] = transforms[parent] @ origin(joint.find("origin"))
            else:
                unresolved.append(joint)
        if len(unresolved) == len(pending):
            raise ValueError("URDF joint tree is disconnected or cyclic")
        pending = unresolved
    results = []
    for link in robot.findall("link"):
        name = link.get("name")
        body = bodies["chassis_link" if name == "base_link" else name]
        usd_points, urdf_points = [], []
        for prim in Usd.PrimRange(body):
            if prim.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(prim)
                transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()) * chassis_inverse
                usd_points.append(transformed(mesh.GetPointsAttr().Get(), np.array(transform).T))
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                raise ValueError("This audit expects exported OBJ visuals")
            uri = mesh.get("filename")
            path = Path(unquote(urlsplit(uri).path)) if uri.startswith("file://") else urdf.parent / uri
            points = [[float(x) for x in line.split()[1:4]] for line in path.read_text().splitlines() if line.startswith("v ")]
            scale = [float(x) for x in mesh.get("scale", "1 1 1").split()]
            urdf_points.append(transformed(np.array(points) * scale, transforms[name] @ origin(visual.find("origin"))))
        usd_box, urdf_box = bounds(usd_points), bounds(urdf_points)
        error = float(np.max(np.abs(usd_box - urdf_box)))
        results.append({"link": name, "source_body": str(body.GetPath()), "usd_bounds_m": usd_box.tolist(),
                        "urdf_bounds_m": urdf_box.tolist(), "maximum_bounds_error_m": error,
                        "matches_1mm": error <= .001})
    return {"source_path": str(source), "urdf_path": str(urdf), "method": "per-link neutral-pose visual AABB in chassis frame",
            "source_saved": False, "all_links_match_1mm": all(x["matches_1mm"] for x in results), "links": results,
            "limits": "Compares visual bounds and neutral-pose placement; does not prove mesh topology or dynamic motion."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.source, args.urdf)
    args.output = private_output(args.output)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        stream.write(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"all_links_match_1mm": result["all_links_match_1mm"], "links": len(result["links"]),
                      "maximum_error_m": max(x["maximum_bounds_error_m"] for x in result["links"])}))
