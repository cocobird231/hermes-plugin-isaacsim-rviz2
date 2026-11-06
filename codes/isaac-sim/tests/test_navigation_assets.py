"""Coordinates and URDF topology are safety-critical conversion boundaries."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1] / "navigation"


def module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / (name + ".py"))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


maths, assets = module("asset_math"), module("assets")


class AssetCoordinateTests(unittest.TestCase):
    def test_nonzero_world_origin_and_png_y_direction(self):
        # Offset rectangular map detects both accidental seed-as-origin and XY transpose.
        pixels, width, height, origin, outside = maths.rasterize_positions(
            [(160.5, -154.5, .6), (162.5, -153.5, .6)], [(160.5, -153.5, .6)],
            (160, -155, .2), (163, -153, 1), 1)
        self.assertEqual((width, height, origin, outside), (3, 2, [160., -155., 0.], 0))
        self.assertEqual(list(pixels), [0, 127, 255, 255, 127, 127])
        self.assertEqual(maths.world_to_pixel(160.5, -154.5, origin, 1, width, height), (1, 0))
        self.assertIsNone(maths.world_to_pixel(163, -154, origin, 1, width, height))

    def test_centimeters_and_occupied_precedence(self):
        pixels, width, height, origin, _ = maths.rasterize_positions(
            [(105, 205, 10)], [(105, 205, 20)], (100, 200, 0), (120, 230, 30), .1, .01)
        self.assertEqual((width, height, origin), (2, 3, [1, 2, 0]))
        self.assertEqual(pixels[4], 0)

    def test_quality_does_not_connect_across_obstacle(self):
        quality = maths.quality_metrics(bytearray([255, 0, 255, 255, 0, 255]), 3, 2, (1, 0))
        self.assertEqual(quality["free_cells"], 4)
        self.assertEqual(quality["seed_connected_free_cells"], 2)
        self.assertTrue(quality["usable_seed_region"])

    def test_mobile_urdf_preserves_physical_joints(self):
        text = '''<robot name="rv1"><link name="world"/><link name="chassis_link"/><link name="wheel"/>
        <joint name="floating_base" type="floating"><parent link="world"/><child link="chassis_link"/></joint>
        <joint name="drive" type="continuous"><parent link="chassis_link"/><child link="wheel"/><axis xyz="0 1 0"/></joint></robot>'''
        converted, details = maths.adapt_urdf_for_rviz(text)
        robot = ET.fromstring(converted)
        self.assertEqual(details["source_base_link"], "chassis_link")
        self.assertEqual(details["movable_joints"], ["drive"])
        self.assertEqual(robot.find("joint/parent").get("link"), "base_link")
        self.assertEqual(robot.find("joint/axis").get("xyz"), "0 1 0")
        self.assertEqual(len(robot.findall("link")), 2)

    def test_request_rejects_seed_outside_and_wrong_gray_threshold(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "map.usd"
            source.touch()
            request = {"operation": "map", "source_path": str(source), "output_dir": directory + "/out",
                       "seed": [160, -155, .5], "bounds_xy": [159, -156, 162, -154], "z_range": [.2, 1]}
            validated = assets.validate_asset_request(request)
            self.assertEqual(validated["seed_stage_xyz_m"], [160, -155, .5])
            with self.assertRaisesRegex(ValueError, "seed XY"):
                assets.validate_asset_request({**request, "seed": [165, -155, .5]})
            with self.assertRaisesRegex(ValueError, "gray=127"):
                assets.validate_asset_request({**request, "occupied_thresh": .4})
            with self.assertRaisesRegex(ValueError, "Unsupported"):
                assets.validate_asset_request({**request, "make_unknown_free": True})

    def test_urdfdom_limits_without_inventing_actuator_bounds(self):
        text = '''<robot name="rv1"><link name="base_link"/><link name="shock"/><link name="wheel"/>
        <joint name="shock_1" type="prismatic"><parent link="base_link"/><child link="shock"/>
        <limit lower="-0.2" upper="0.2" velocity="inf"/></joint>
        <joint name="drive_1" type="revolute"><parent link="shock"/><child link="wheel"/>
        <limit lower="-inf" upper="inf" velocity="17.45"/></joint></robot>'''
        converted, details = maths.adapt_urdf_for_rviz(text, joint_efforts={"shock_1": 20000, "drive_1": 1200})
        robot = ET.fromstring(converted)
        self.assertEqual(robot.find("joint[@name='shock_1']/limit").attrib,
                         {"lower": "-0.2", "upper": "0.2", "velocity": "0", "effort": "20000"})
        self.assertEqual(robot.find("joint[@name='drive_1']").get("type"), "continuous")
        self.assertEqual(details["limit_normalization"][0]["velocity_placeholder"]["scope"], "visualization_only")

    def test_height_projection_requires_all_free_and_any_occupied_blocks(self):
        result = maths.project_volume(
            [(0.5, .5, .5), (.5, .5, 1.5), (1.5, .5, .5), (2.5, .5, .5), (2.5, .5, 1.5)],
            [(2.5, .5, 1.5)], (0, 0, 0), (3, 1, 2), 1)
        self.assertEqual(list(result[0]), [255, 127, 0])
        self.assertEqual(result[-1]["height_cells"], 2)
        self.assertEqual(result[-1]["voxel_fractional_offsets_from_min_bound"], [[.5], [.5], [.5]])

    def test_volume_budget_rejects_large_height_even_when_xy_small(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "map.usd"
            source.touch()
            request = {"operation": "map", "source_path": str(source), "output_dir": directory + "/out",
                       "seed": [1, 1, 1], "bounds_xy": [0, 0, 2, 2], "z_range": [0, 10000],
                       "resolution_m": .01}
            with self.assertRaisesRegex(ValueError, "max_voxels"):
                assets.validate_asset_request(request)

    def test_full_scene_default_uses_all_bounds_and_preserves_world_coordinates(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "gongguan.usdz"
            source.touch()
            request = assets.validate_asset_request({
                "operation": "map", "source_path": str(source), "output_dir": directory + "/out",
                "seed_stage_xyz_m": [160, -155, .8], "z_min_m": .3, "z_max_m": 1.8,
                "resolution_m": .2, "max_voxels": 40_000_000})
            self.assertEqual(request["bounds_mode"], "full_scene")
            bounds, budget = assets.resolve_map_bounds(
                request, [-93.7624969482, -220.2190246582, -1.42715], [352.0039978027, 30.1649589539, 27.225])
            for actual, expected in zip(bounds, [-93.8, -220.4, 352.2, 30.2]):
                self.assertAlmostEqual(actual, expected)
            self.assertEqual(budget["requested_cells"], 2230 * 1253)
            self.assertNotIn("bounds_stage_xy_m", request)
            self.assertGreater(budget["native_padding_budget_voxels"], budget["requested_volume_cells"])

    def test_full_scene_budget_is_checked_after_discovery_not_silently_cropped(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "map.usd"
            source.touch()
            request = assets.validate_asset_request({
                "operation": "map", "source_path": str(source), "output_dir": directory + "/out",
                "seed": [1, 1, .8], "z_range": [.3, 1.8], "resolution_m": .1,
                "max_cells": 16_000_000, "max_voxels": 64_000_000})
            with self.assertRaisesRegex(ValueError, "max_voxels"):
                assets.resolve_map_bounds(request, [-94, -221, -2], [353, 31, 28])
            with self.assertRaisesRegex(ValueError, "finite"):
                assets.resolve_map_bounds(request, [float("nan"), 0, 0], [10, 10, 2])

    def test_explicit_bounds_and_padding_cannot_conflict_with_full_scene(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "map.usd"
            source.touch()
            request = {"operation": "map", "source_path": str(source), "output_dir": directory + "/out",
                       "seed": [1, 1, .8], "z_range": [.3, 1.8], "bounds_xy": [0, 0, 2, 2]}
            self.assertEqual(assets.validate_asset_request(request)["bounds_mode"], "explicit")
            with self.assertRaisesRegex(ValueError, "omit bounds"):
                assets.validate_asset_request({**request, "bounds_mode": "full_scene"})
            with self.assertRaisesRegex(ValueError, "only to full_scene"):
                assets.validate_asset_request({**request, "scene_padding_m": 1})

    def test_full_scene_padding_is_outward_and_seed_outside_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "map.usd"
            source.touch()
            request = assets.validate_asset_request({
                "operation": "map", "source_path": str(source), "output_dir": directory + "/out",
                "seed": [1, 1, .8], "z_range": [.3, 1.8], "resolution_m": .2, "scene_padding_m": .3})
            bounds, _ = assets.resolve_map_bounds(request, [.15, .15, 0], [1.95, 1.95, 2])
            for actual, expected in zip(bounds, [-.2, -.2, 2.4, 2.4]):
                self.assertAlmostEqual(actual, expected)
            with self.assertRaisesRegex(ValueError, "seed XY"):
                assets.resolve_map_bounds(request, [10, 10, 0], [20, 20, 2])

    def test_volume_duplicate_samples_never_fill_unknown_layers(self):
        result = maths.project_volume(
            [(0.5, .5, .5)] * 10 + [(1.5, .5, .5), (1.5, .5, 1.5), (2.5, .5, .5), (2.5, .5, 1.5)],
            [(2.5, .5, 1.5)], (0, 0, 0), (3, 1, 2), 1)
        self.assertEqual(list(result[0]), [127, 255, 0])


if __name__ == "__main__":
    unittest.main()
