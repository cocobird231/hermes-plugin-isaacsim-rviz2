"""Select Isaac 6 bundled Jazzy libraries before the ELF loader starts."""
from pathlib import Path


def configure_ros_libraries(python, environment):
    explicit = environment.get("HERMES_ISAAC_ROS_LIB_DIR")
    if explicit:
        candidates = [Path(explicit).expanduser()]
    else:
        python = Path(python)
        # Standalone distribution and pip venv installations respectively.
        candidates = [python.parent / "exts/isaacsim.ros2.core/jazzy/lib"]
        candidates += sorted(python.parent.parent.glob(
            "lib/python*/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy/lib"))
    for candidate in candidates:
        if (candidate / "librcl.so").is_file():
            environment["LD_LIBRARY_PATH"] = str(candidate.resolve())
            environment["ROS_DISTRO"] = "jazzy"
            environment.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
            return str(candidate.resolve())
    if explicit:
        raise ValueError("HERMES_ISAAC_ROS_LIB_DIR does not contain the Isaac bundled librcl.so")
    return None  # Legacy lifecycle can still work on installations without ROS.
