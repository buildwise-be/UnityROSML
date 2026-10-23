"""Focused checks for frame composition and the deliberately restricted parser."""

import math
from pathlib import Path
import unittest
from unittest.mock import patch

from check_robot_frames import (
    ARM_SECTIONS, calibration_yaml, chain, compare_fixed_chains, compare_transforms, compose,
    estimated_tool0_tcp, identity, inverse, multiply, transform,
)


class FrameMathTests(unittest.TestCase):
    def test_urdf_rpy_composition_rotates_child_translation(self):
        parent = transform([1, 2, 3], [0, 0, math.pi / 2])
        child = transform([1, 0, 0], [0, 0, 0])
        result = multiply(parent, child)
        for actual, expected in zip([result[i][3] for i in range(3)], [1, 3, 3]):
            self.assertAlmostEqual(actual, expected)
        self.assertLess(compare_transforms(multiply(result, inverse(result)), identity())
                        ["position_error_mm"], 1e-9)

    def test_joint_motion_occurs_after_origin_transform(self):
        joints = [
            dict(name="hinge", type="revolute", parent="base", child="arm",
                 xyz_m=[0, 0, 0], rpy_rad=[0, 0, 0], axis=[0, 0, 1]),
            dict(name="tip", type="fixed", parent="arm", child="tcp",
                 xyz_m=[2, 0, 0], rpy_rad=[0, 0, 0], axis=[1, 0, 0]),
        ]
        result = compose(chain(joints, "base", "tcp"), {"hinge": math.pi / 2})
        self.assertAlmostEqual(result[0][3], 0)
        self.assertAlmostEqual(result[1][3], 2)
        with self.assertRaisesRegex(ValueError, "movable"):
            chain(joints, "base", "tcp", require_fixed=True)

    def test_rotation_distance_handles_wrap_and_tiny_differences(self):
        a = transform([0, 0, 0], [0, 0, math.pi])
        b = transform([0, 0, 0], [0, 0, -math.pi])
        self.assertLess(compare_transforms(a, b)["orientation_error_degrees"], 1e-10)
        c = transform([0, 0, 0], [0, 0, 1e-7])
        self.assertAlmostEqual(compare_transforms(identity(), c)["orientation_error_degrees"],
                               math.degrees(1e-7), places=10)

    def test_cancelled_offsets_do_not_hide_different_tool_mounts(self):
        unity = [
            dict(name="mount", type="fixed", parent="flange", child="body",
                 xyz_m=[1, 0, 0], rpy_rad=[0, 0, 0], axis=[1, 0, 0]),
            dict(name="tip", type="fixed", parent="body", child="tcp",
                 xyz_m=[-1, 0, 0], rpy_rad=[0, 0, 0], axis=[1, 0, 0]),
        ]
        ros = [dict(joint, xyz_m=[0, 0, 0]) for joint in unity]
        self.assertLess(compare_transforms(compose(unity), compose(ros))["position_error_mm"], 1e-9)
        self.assertFalse(compare_fixed_chains(unity, ros)["matches_old_ros_default"])

    def test_calibration_parser_rejects_incomplete_numeric_sections(self):
        content = "kinematics:\n"
        for section in ARM_SECTIONS.values():
            content += f"  {section}:\n"
            for key in ("x", "y", "z", "roll", "pitch", "yaw"):
                content += f"    {key}: 0\n"
        content += "  hash: example_hash\n"
        path = Path("calibration.yaml")
        with patch.object(Path, "read_text", return_value=content):
            blocks, calibration_hash = calibration_yaml(path)
            self.assertEqual(len(blocks), 6)
            self.assertEqual(calibration_hash, "example_hash")
        with patch.object(Path, "read_text", return_value=content.replace("    yaw: 0\n", "", 1)):
            with self.assertRaisesRegex(ValueError, "Incomplete"):
                calibration_yaml(path)

    def test_estimated_tool0_depth_uses_rotated_urdf_axis(self):
        joints = [
            dict(name="tool0", type="fixed", parent="ur_flange", child="ur_tool0",
                 xyz_m=[0, 0, 0], rpy_rad=[math.pi / 2, 0, math.pi / 2], axis=[1, 0, 0]),
            dict(name="old_tcp", type="fixed", parent="ur_flange", child="tcp",
                 xyz_m=[0.11, 0, 0], rpy_rad=[-math.pi / 2, 0, -math.pi / 2], axis=[1, 0, 0]),
        ]
        result = estimated_tool0_tcp(joints, 150)
        for actual, expected in zip(result["flange_to_candidate_tcp"]["translation_m"], [0.15, 0, 0]):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(result["candidate_minus_old_kenos"]["position_error_mm"], 40)
        self.assertAlmostEqual(result["candidate_minus_old_kenos"]["orientation_error_degrees"], 180)
        self.assertEqual(result["status"], "UNMEASURED_ESTIMATE_NOT_APPLIED")
        for invalid in (0, -1, math.inf, math.nan):
            with self.assertRaisesRegex(ValueError, "finite and greater"):
                estimated_tool0_tcp(joints, invalid)


if __name__ == "__main__":
    unittest.main()
