"""Robot identity, endpoint and footprint configuration without ROS/Isaac."""
import copy
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rviz_navigation.profiles import default_rv1_profile, nav2_parameters, navigation_ids, validate_profile

PROJECT = Path(__file__).resolve().parents[3]


@pytest.fixture
def rover(tmp_path):
    # Reuse complete Nav2 plugin declarations, but give this robot different
    # geometry, endpoint names, clock mode and no Isaac backend dependency.
    content = yaml.safe_load((PROJECT / "configs/rv1_nav2.yaml").read_text())
    for name in ("global_costmap", "local_costmap"):
        block = content[name][name]["ros__parameters"]
        block["footprint"] = "[[-0.3,-0.2],[0.3,-0.2],[0.3,0.2],[-0.3,0.2]]"
        block["footprint_padding"] = 0.02
    path = tmp_path / "rover-nav2.yaml"
    path.write_text(yaml.safe_dump(content))
    return {"robot_id": "inspection_rover", "frames": {"map": "site_map", "odom": "rover_odom", "base": "chassis"},
            "topics": {name: "/rover/" + name for name in ("map", "odom", "cmd_vel", "joint_states", "robot_description", "clock")},
            "use_sim_time": False, "params_file": str(path),
            "localization_mode": "tf_verified", "control_backend": "ros2"}


def test_custom_robot_profile_rewrites_all_robot_specific_nav2_endpoints(rover):
    original = copy.deepcopy(rover)
    profile = validate_profile(rover)
    assert rover == original
    content = nav2_parameters(profile)
    for name, global_frame in (("global_costmap", "site_map"), ("local_costmap", "rover_odom")):
        block = content[name][name]["ros__parameters"]
        assert block["robot_base_frame"] == "chassis"
        assert block["global_frame"] == global_frame
        assert block["static_layer"]["map_topic"] == "/rover/map"
        assert not block["use_sim_time"]
        assert block["footprint_padding"] == 0.02
        assert "0.3" in block["footprint"]
    for name in ("controller_server", "bt_navigator", "velocity_smoother"):
        assert content[name]["ros__parameters"]["odom_topic"] == "/rover/odom"
    assert profile["control_backend"] == "ros2"


def test_builtin_rv1_is_explicit_and_keeps_reviewed_geometry():
    profile = default_rv1_profile(PROJECT)
    assert profile["robot_id"] == "rv1" and profile["control_backend"] == "isaac_rv1"
    assert profile["limits"]["max_yaw_rps"] == 0.5
    assert "1.421" in nav2_parameters(profile)["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"]


@pytest.mark.parametrize("mutate", [
    lambda p: p.update(robot_id="../rv1"),
    lambda p: p["frames"].update(base="/chassis"),
    lambda p: p["frames"].update(base="site_map"),
    lambda p: p["topics"].update(cmd_vel="/rover/odom"),
    lambda p: p["topics"].update(cmd_vel="cmd_vel"),
    lambda p: p["topics"].update(cmd_vel="/rviz_navigation/inspection_rover/cmd_vel_smoothed"),
    lambda p: p.update(localization_mode="amcl"),
    lambda p: p.update(use_sim_time=1),
    lambda p: p.update(limits={"max_linear_mps": float("nan")}),
    lambda p: p.update(limits={"command_timeout_seconds": 0}),
    lambda p: p.update(footprint=[[0, 0], [1, 1], [2, 2]]),
    lambda p: p.update(footprint_padding_m=-1),
    lambda p: p.update(control_backend="arbitrary_python"),
    lambda p: p.pop("params_file"),
])
def test_invalid_or_ambiguous_profiles_are_rejected(rover, mutate):
    mutate(rover)
    with pytest.raises(ValueError):
        validate_profile(rover)


def test_custom_footprint_overrides_both_costmaps_without_modifying_original_file(rover):
    path = Path(rover["params_file"])
    before = path.read_bytes()
    rover.update(footprint=[[-0.5, -0.4], [0.5, -0.4], [0.5, 0.4], [-0.5, 0.4]], footprint_padding_m=0.04)
    content = nav2_parameters(validate_profile(rover))
    blocks = [content[name][name]["ros__parameters"] for name in ("global_costmap", "local_costmap")]
    assert blocks[0]["footprint"] == blocks[1]["footprint"]
    assert blocks[0]["footprint_padding"] == blocks[1]["footprint_padding"] == 0.04
    assert path.read_bytes() == before


def test_missing_footprint_never_falls_back_to_rv1(rover):
    path = Path(rover["params_file"])
    content = yaml.safe_load(path.read_text())
    del content["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"]
    path.write_text(yaml.safe_dump(content))
    with pytest.raises(ValueError, match="footprint"):
        validate_profile(rover)


def test_nav2_plugin_ids_come_from_configured_robot(rover):
    path = Path(rover["params_file"])
    content = yaml.safe_load(path.read_text())
    content["planner_server"]["ros__parameters"]["planner_plugins"] = ["SitePlanner"]
    content["controller_server"]["ros__parameters"]["controller_plugins"] = ["RoverController"]
    content["controller_server"]["ros__parameters"]["goal_checker_plugins"] = ["ArrivalCheck"]
    for node, old, new in (("planner_server", "GridBased", "SitePlanner"),
                           ("controller_server", "FollowPath", "RoverController"),
                           ("controller_server", "goal_checker", "ArrivalCheck")):
        block = content[node]["ros__parameters"]
        block[new] = block.pop(old)
    path.write_text(yaml.safe_dump(content))
    assert navigation_ids(nav2_parameters(validate_profile(rover))) == {
        "planner": "SitePlanner", "controller": "RoverController", "goal_checker": "ArrivalCheck"}


def test_renamed_static_map_plugin_uses_configured_map_topic(rover):
    path = Path(rover["params_file"])
    content = yaml.safe_load(path.read_text())
    for name in ("global_costmap", "local_costmap"):
        block = content[name][name]["ros__parameters"]
        block["site_static_map"] = block.pop("static_layer")
        block["plugins"] = ["site_static_map", "inflation_layer"]
    path.write_text(yaml.safe_dump(content))
    active = nav2_parameters(validate_profile(rover))
    for name in ("global_costmap", "local_costmap"):
        block = active[name][name]["ros__parameters"]
        assert block["site_static_map"]["map_topic"] == "/rover/map"
        assert "static_layer" not in block


def test_maximum_speed_opt_in_preserves_baseline_geometry_and_arrival():
    from rviz_navigation.profiles import maximum_speed_rv1_profile
    baseline = default_rv1_profile(PROJECT)
    maximum = maximum_speed_rv1_profile(PROJECT)
    assert baseline['limits']['max_linear_mps'] == 1.0
    assert maximum['limits']['max_linear_mps'] == 2.5
    old, new = nav2_parameters(baseline), nav2_parameters(maximum)
    follow = new['controller_server']['ros__parameters']['FollowPath']
    assert follow['max_vel_x'] == follow['max_speed_xy'] == 2.5
    assert new['velocity_smoother']['ros__parameters']['max_velocity'] == [2.5, 0.0, 0.5]
    assert follow['acc_lim_x'] == 2.0 and follow['decel_lim_x'] == -3.0
    for name in ('global_costmap', 'local_costmap'):
        for field in ('footprint', 'footprint_padding', 'inflation_layer'):
            assert old[name][name]['ros__parameters'][field] == new[name][name]['ros__parameters'][field]
    assert old['controller_server']['ros__parameters']['goal_checker'] == new['controller_server']['ros__parameters']['goal_checker']
    assert old['controller_server']['ros__parameters']['FollowPath']['max_vel_x'] == .4
    assert new['local_costmap']['local_costmap']['ros__parameters']['width'] / 2 > 2.5 * follow['sim_time'] + 2.56
    assert follow['forward_prune_distance'] > 2.5 * follow['sim_time']
