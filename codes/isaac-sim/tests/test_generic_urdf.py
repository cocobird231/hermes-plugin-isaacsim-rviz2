"""Generic robot conversion rejects incomplete topology and preserves identities."""
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1] / "navigation"
sys.path.insert(0, str(ROOT))
from asset_math import adapt_urdf_for_rviz
from asset_worker import select_robot_subtree
from assets import validate_asset_request

try:
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics
except ImportError:
    Usd = None


class GenericUrdfTests(unittest.TestCase):
    def test_request_robot_name_and_defaults_are_not_rv1(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "cart.usda"
            source.touch()
            request = {"operation": "urdf", "source_path": str(source), "output_dir": directory + "/out"}
            self.assertEqual(validate_asset_request(request)["robot_name"], "robot")
            data = validate_asset_request({**request, "robot_name": "inspection_cart", "base_frame": "cart_base"})
            self.assertEqual(data["robot_name"], "inspection_cart")
            self.assertEqual(data["base_frame"], "cart_base")
            with self.assertRaisesRegex(ValueError, "robot_name"):
                validate_asset_request({**request, "robot_name": "../outside"})

    def test_non_rv1_name_base_and_kinematics(self):
        raw = '''<robot name="inspection_cart"><link name="world"/><link name="chassis"/>
        <link name="camera"/><joint name="floating_base" type="floating">
        <parent link="world"/><child link="chassis"/></joint>
        <joint name="camera_mount" type="fixed"><parent link="chassis"/><child link="camera"/>
        <origin xyz="0.3 0 0.4" rpy="0 0 0"/></joint></robot>'''
        text, details = adapt_urdf_for_rviz(raw, "cart_base", {}, "inspection_cart_2")
        robot = ET.fromstring(text)
        self.assertEqual(robot.get("name"), "inspection_cart_2")
        self.assertEqual(details["source_robot_name"], "inspection_cart")
        self.assertEqual(details["source_base_link"], "chassis")
        self.assertEqual(details["topology"], "single_connected_tree")
        self.assertEqual(robot.find("joint/parent").get("link"), "cart_base")
        self.assertEqual(robot.find("joint/origin").get("xyz"), "0.3 0 0.4")

    def test_world_geometry_is_never_silently_deleted(self):
        raw = '''<robot name="cart"><link name="world"><visual/></link><link name="base"/>
        <joint name="floating_base" type="floating"><parent link="world"/><child link="base"/></joint></robot>'''
        with self.assertRaisesRegex(ValueError, "physical or ambiguous world"):
            adapt_urdf_for_rviz(raw)

    def test_missing_link_rejected(self):
        raw = '''<robot name="cart"><link name="base"/><joint name="bad" type="fixed">
        <parent link="base"/><child link="missing"/></joint></robot>'''
        with self.assertRaisesRegex(ValueError, "missing link"):
            adapt_urdf_for_rviz(raw)

    def test_disconnected_cycle_rejected_even_with_one_root(self):
        raw = '''<robot name="cart"><link name="base"/><link name="a"/><link name="b"/>
        <joint name="a_b" type="fixed"><parent link="a"/><child link="b"/></joint>
        <joint name="b_a" type="fixed"><parent link="b"/><child link="a"/></joint></robot>'''
        with self.assertRaisesRegex(ValueError, "disconnected or cyclic"):
            adapt_urdf_for_rviz(raw)

    def test_multiple_parents_rejected(self):
        raw = '''<robot name="cart"><link name="base"/><link name="a"/><link name="b"/>
        <joint name="a_b" type="fixed"><parent link="a"/><child link="b"/></joint>
        <joint name="base_b" type="fixed"><parent link="base"/><child link="b"/></joint></robot>'''
        with self.assertRaisesRegex(ValueError, "multiple parents"):
            adapt_urdf_for_rviz(raw)

    def test_unknown_effort_is_rejected_not_invented(self):
        raw = '''<robot name="cart"><link name="base"/><link name="wheel"/>
        <joint name="wheel" type="revolute"><parent link="base"/><child link="wheel"/>
        <limit lower="-inf" upper="inf" velocity="12"/></joint></robot>'''
        with self.assertRaisesRegex(ValueError, "no finite USD drive maxForce"):
            adapt_urdf_for_rviz(raw, joint_efforts={})

    def test_native_continuous_joint_omitted_limits_use_real_effort(self):
        raw = '''<robot name="cart"><link name="base"/><link name="wheel"/>
        <joint name="wheel" type="continuous"><parent link="base"/><child link="wheel"/></joint></robot>'''
        converted, details = adapt_urdf_for_rviz(raw, joint_efforts={"wheel": 8})
        limit = ET.fromstring(converted).find("joint/limit")
        self.assertEqual(limit.attrib, {"effort": "8", "velocity": "0"})
        self.assertTrue(details["limit_normalization"][0]["added_continuous_joint_limit"])

    def test_nonfinite_prismatic_bounds_rejected(self):
        raw = '''<robot name="cart"><link name="base"/><link name="slider"/>
        <joint name="slider" type="prismatic"><parent link="base"/><child link="slider"/>
        <limit lower="-inf" upper="inf" velocity="1" effort="10"/></joint></robot>'''
        with self.assertRaisesRegex(ValueError, "finite lower and upper"):
            adapt_urdf_for_rviz(raw, joint_efforts={})


@unittest.skipIf(Usd is None, "USD topology tests run in the host Isaac Python environment")
class UsdRobotSelectionTests(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()

    def robot(self, path):
        root = UsdGeom.Xform.Define(self.stage, path).GetPrim()
        UsdPhysics.ArticulationRootAPI.Apply(root)
        for name in ("chassis", "wheel"):
            body = UsdGeom.Xform.Define(self.stage, path + "/" + name).GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(body)
        joint = UsdPhysics.RevoluteJoint.Define(self.stage, path + "/wheel_joint")
        joint.CreateBody0Rel().SetTargets([path + "/chassis"])
        joint.CreateBody1Rel().SetTargets([path + "/wheel"])
        return root

    def test_default_selects_non_rv1_and_preserves_stage(self):
        root = self.robot("/Cart")
        self.stage.SetDefaultPrim(root)
        before = self.stage.GetRootLayer().ExportToString()
        selected, joints, details = select_robot_subtree(self.stage)
        self.assertEqual(str(selected.GetPath()), "/Cart")
        self.assertEqual(details["articulation_prim_path"], "/Cart")
        self.assertEqual(len(joints), 1)
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), before)

    def test_multiple_robots_require_selection_and_scope(self):
        self.robot("/CartA")
        self.robot("/CartB")
        with self.assertRaisesRegex(ValueError, "exactly one articulation"):
            select_robot_subtree(self.stage)
        _, joints, details = select_robot_subtree(self.stage, "/CartB")
        self.assertEqual(details["rigid_body_paths"], ["/CartB/chassis", "/CartB/wheel"])
        self.assertEqual([str(j.GetPath()) for j in joints], ["/CartB/wheel_joint"])

    def test_existing_wrong_subtree_never_falls_back(self):
        self.robot("/Cart")
        UsdGeom.Xform.Define(self.stage, "/Decoration")
        with self.assertRaisesRegex(ValueError, "stage-wide fallback is disabled"):
            select_robot_subtree(self.stage, "/Decoration")

    def test_cross_robot_joint_rejected(self):
        self.robot("/CartA")
        self.robot("/CartB")
        joint = UsdPhysics.Joint(self.stage.GetPrimAtPath("/CartA/wheel_joint"))
        joint.GetBody1Rel().SetTargets(["/CartB/wheel"])
        with self.assertRaisesRegex(ValueError, "outside root_prim_path"):
            select_robot_subtree(self.stage, "/CartA")

    def test_disconnected_body_is_not_silently_omitted(self):
        self.robot("/Cart")
        stray = UsdGeom.Xform.Define(self.stage, "/Cart/stray").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(stray)
        with self.assertRaisesRegex(ValueError, "connected acyclic"):
            select_robot_subtree(self.stage, "/Cart")

    def test_closed_chain_rejected(self):
        self.robot("/Cart")
        joint = UsdPhysics.Joint(self.stage.GetPrimAtPath("/Cart/wheel_joint"))
        joint.CreateExcludeFromArticulationAttr(True)
        with self.assertRaisesRegex(ValueError, "Closed-chain"):
            select_robot_subtree(self.stage, "/Cart")


if __name__ == "__main__":
    unittest.main()
