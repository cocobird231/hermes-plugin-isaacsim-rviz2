import pytest

from navigation.launch_environment import configure_ros_libraries


def test_pip_jazzy_libs_are_selected_before_launch(tmp_path):
    library = tmp_path / "lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy/lib"
    library.mkdir(parents=True)
    (library / "librcl.so").touch()
    env = {"ROS_DOMAIN_ID": "37"}
    assert configure_ros_libraries(tmp_path / "bin/python", env) == str(library)
    assert env == {"ROS_DOMAIN_ID": "37", "ROS_DISTRO": "jazzy",
                   "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp", "LD_LIBRARY_PATH": str(library)}


def test_invalid_explicit_directory_is_not_silently_ignored(tmp_path):
    with pytest.raises(ValueError):
        configure_ros_libraries(tmp_path / "python", {"HERMES_ISAAC_ROS_LIB_DIR": str(tmp_path)})


def test_legacy_install_without_bridge_remains_launchable(tmp_path):
    env = {"DISPLAY": ":1"}
    assert configure_ros_libraries(tmp_path / "python", env) is None
    assert env == {"DISPLAY": ":1"}
