"""Isaac asset tools and RV1 ROS adapter; schema imports do not load ROS, USD or Isaac."""
PATH = {"type": "string", "description": "Absolute path on the Isaac host."}
NUMBER = {"type": "number"}
POSE = {"x_m": NUMBER, "y_m": NUMBER, "yaw_rad": NUMBER}
VECTOR3 = {"type": "array", "items": NUMBER, "minItems": 3, "maxItems": 3}
VECTOR4 = {"type": "array", "items": NUMBER, "minItems": 4, "maxItems": 4}

# (public tool name, authenticated bridge action, description, fields, required)
TOOLS = [
    ("isaac_sim_export_occupancy_map", "nav_export_map",
     "Export a local USD/USDZ to PNG + ROS map YAML in a separate Isaac worker. Omit bounds_stage_xy_m for the entire enabled collision scene; an explicit rectangle exports only that ROI. Unmeshed Gaussian volumes cannot supply PhysX occupancy. The managed interactive Isaac must be closed. Returns a job_id; use isaac_sim_export_status. Inspect geometry/unknown/seed quality and fixed-height ground limitations before navigation.",
     {"source_path": PATH, "output_dir": PATH,
      "resolution_m": {"type": "number", "minimum": .005, "maximum": 2, "default": .1},
      "seed_stage_xyz_m": VECTOR3, "bounds_stage_xy_m": VECTOR4,
      "bounds_mode": {"type": "string", "enum": ["full_scene", "explicit"],
                      "description": "Inferred: full_scene when bounds are omitted, explicit when supplied. full_scene rejects explicit bounds."},
      "scene_padding_m": {"type": "number", "minimum": 0, "maximum": 100, "default": 0,
                          "description": "Outward padding around full-scene geometry before snapping to the resolution grid."},
      "z_min_m": NUMBER, "z_max_m": NUMBER,
      "geometry_mode": {"type": "string", "enum": ["physx", "visual_meshes"], "default": "physx"},
      "exclude_prim_paths": {"type": "array", "items": {"type": "string"}},
      "free_thresh": NUMBER, "occupied_thresh": NUMBER,
      "max_cells": {"type": "integer", "minimum": 1, "maximum": 16000000},
      "max_voxels": {"type": "integer", "minimum": 1, "maximum": 64000000}},
     ["source_path", "seed_stage_xyz_m", "z_min_m", "z_max_m"]),
    ("isaac_sim_export_robot_urdf", "nav_export_urdf",
     "Export a selected articulated robot USD into raw URDF, RViz-compatible URDF and meshes without saving source layers. Requires interactive Isaac closed. Returns a job_id; conversion does not prove RViz display or navigation.",
     {"source_path": PATH, "output_dir": PATH, "root_prim_path": {"type": "string"},
      "base_frame": {"type": "string", "default": "base_link"},
      "robot_name": {"type": "string", "pattern": "^[A-Za-z][A-Za-z0-9_]{0,63}$", "default": "robot"}}, ["source_path"]),
    ("isaac_sim_export_status", "nav_job_status",
     "Poll an export job. Completion requires pending=false, success=true and clean worker exit. Inspect map quality and source_unchanged, not only whether files exist.",
     {"job_id": {"type": "string"}}, ["job_id"]),
    ("isaac_sim_prepare_rv1_ros2", "rv1_ros2_prepare",
     "Prepare continuous Twist /cmd_vel control and measured odom/TF/joint feedback in an already loaded RV1, with simulation stopped. Defaults to explicitly labeled simulation-truth localization, not AMCL. Starts disarmed; never creates a second RV1 or changes the source file. Reprepare after stop/reset.",
     {"config": {"type": "object", "description": "RV1 ROS adapter configuration from the bundled manual: topic/frame names, limited speed, watchdog, sim_truth alignment. Unknown keys are rejected."},
      "wait_seconds": {"type": "number", "minimum": 0, "maximum": 60}}, []),
]
