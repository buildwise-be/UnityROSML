"""Portable regression checks for real expanded model data and runtime verification."""

import copy
import hashlib
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from check_robot_frames import chain, compare_transforms, compose, inverse, multiply, parse_joint, transform
from export_ros_models import matrix_pose, parse_model
from verify_runtime_models import pose_error, verify


RESOURCES = Path(__file__).resolve().parents[2] / "Assets/Resources/RobotModels"


def runtime_copy(catalog):
    return {"models": [{"tool": model["tool"], "cases": copy.deepcopy(model["goldenCases"])}
                       for model in catalog["models"]]}


class ExpandedModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.catalog = json.loads((RESOURCES / "Models.json").read_text())

    def test_catalog_really_reconstructs_from_retained_expanded_xml(self):
        for model in self.catalog["models"]:
            xml_text = (RESOURCES / f"{model['tool']}.urdf.txt").read_text().removesuffix("\n")
            regenerated = parse_model(xml_text, model["tool"], model["calibrationHash"])
            self.assertEqual(regenerated["links"], model["links"])
            for regenerated_case, stored_case in zip(regenerated["goldenCases"], model["goldenCases"]):
                self.assertEqual(regenerated_case["name"], stored_case["name"])
                self.assertEqual(regenerated_case["jointPositions"], stored_case["jointPositions"])
                for regenerated_pose, stored_pose in zip(regenerated_case["links"], stored_case["links"]):
                    position, angle = pose_error(regenerated_pose, stored_pose)
                    self.assertLess(position, 1e-12)  # Windows/Linux libm may differ in final bits.
                    self.assertLess(angle, 1e-10)
            self.assertEqual(hashlib.sha256(xml_text.encode()).hexdigest(), model["expandedUrdfSha256"])

    def test_every_arm_and_tool_frame_is_retained_in_topological_order(self):
        for model in self.catalog["models"]:
            visited = set()
            for link in model["links"]:
                self.assertNotIn(link["name"], visited)
                if visited:
                    self.assertIn(link["parent"], visited)
                else:
                    self.assertEqual(link["name"], "ur_base_link")
                    self.assertEqual(link["parent"], "")
                visited.add(link["name"])
            self.assertTrue({"ur_base_link_inertia", "ur_base", "ur_ft_frame", "ur_flange", "ur_tool0", "tcp"} <= visited)
            self.assertEqual(sum(link["jointType"] == "revolute" for link in model["links"]), 6)
            self.assertEqual(len(visited), 13 if model["tool"] == "none" else 15)

    def test_arm_uses_real_stored_calibration_instead_of_nominal_geometry(self):
        for model in self.catalog["models"]:
            links = {link["name"]: link for link in model["links"]}
            self.assertEqual(model["calibrationHash"], "calib_6927668154486911702")
            self.assertAlmostEqual(links["ur_shoulder_link"]["position"][2], 0.18081968955798347)
            self.assertAlmostEqual(links["ur_forearm_link"]["position"][0], -0.47847653170832138)
            self.assertAlmostEqual(links["ur_wrist_1_link"]["position"][1], -0.0003621142591738172)

    def test_tool_chains_remain_distinct_from_tool0_and_each_other(self):
        transforms = {}
        for model in self.catalog["models"]:
            root = ET.parse(RESOURCES / f"{model['tool']}.urdf.txt").getroot()
            joints = [parse_joint(joint) for joint in root.findall("joint")]
            transforms[model["tool"]] = compose(chain(joints, "ur_flange", "tcp", require_fixed=True))
        self.assertLess(compare_transforms(transforms["none"], transform([0, 0, 0], [0, 0, 0]))["position_error_mm"], 1e-12)
        self.assertAlmostEqual(transforms["kenos_sl"][0][3], 0.15, places=12)
        self.assertAlmostEqual(transforms["kenos_sl"][2][3], -0.034, places=12)
        self.assertGreater(compare_transforms(transforms["kenos"], transforms["kenos_sl"])["position_error_mm"], 40)
        # The no-tool contract is flange identity, not tool0 identity.
        self.assertAlmostEqual(compare_transforms(transforms["kenos"], transforms["kenos_sl"])
                               ["orientation_error_degrees"], 90, delta=0.001)

    def test_all_link_goldens_match_independent_expanded_urdf_chain_composition(self):
        for model in self.catalog["models"]:
            root = ET.parse(RESOURCES / f"{model['tool']}.urdf.txt").getroot()
            joints = [parse_joint(joint) for joint in root.findall("joint")]
            self.assertEqual(len(model["goldenCases"]), 21)
            for case in model["goldenCases"]:
                positions = dict(zip(case["jointNames"], case["jointPositions"]))
                for pose in case["links"]:
                    expected = compose(chain(joints, "ur_base_link", pose["name"]), positions)
                    position, angle = pose_error(matrix_pose(expected), pose)
                    self.assertLess(position, 1e-12)
                    self.assertLess(angle, 1e-10)

    def test_safe_light_geometry_preserves_eccentric_pad_and_cylinder(self):
        model = next(model for model in self.catalog["models"] if model["tool"] == "kenos_sl")
        links = {link["name"]: link for link in model["links"]}
        mount = links["kenos_sl_mount"]["visuals"][0]
        self.assertEqual(mount["kind"], "cylinder")
        self.assertEqual(mount["length"], 0.06025)
        self.assertEqual(mount["radius"], 0.05)
        pad = links["kenos_sl_base"]["visuals"][2]
        self.assertEqual(pad["size"], [0.240, 0.120, 0.024])
        self.assertEqual(pad["position"], [0.034, 0.0, 0.07775])
        for link in (links["kenos_sl_mount"], links["kenos_sl_base"]):
            self.assertEqual(len(link["visuals"]), len(link["collisions"]))
            for visual, collision in zip(link["visuals"], link["collisions"]):
                for key in ("kind", "position", "rotation", "size", "radius", "length"):
                    self.assertEqual(visual[key], collision[key])

    def test_provenance_includes_consumed_arm_parameters_and_current_tool_macros(self):
        sources = {item["path"]: item["sha256"] for item in self.catalog["sourceFiles"]}
        for path in ("ur_description/urdf/ur_macro.xacro", "ur_description/urdf/inc/ur_common.xacro",
                     "ur_description/config/ur16e/visual_parameters.yaml",
                     "ros/ros_ws/src/ur16e_mir250_control/config/ur16e_calibration.yaml",
                     "ros/ros_ws/src/kenos-flexigrip/urdf/kenos.urdf.xacro",
                     "ros/ros_ws/src/kenos-flexigrip/urdf/kenos_sl.urdf.xacro"):
            self.assertIn(path, sources)
            self.assertEqual(len(sources[path]), 64)

    def test_rotation_export_survives_each_half_turn_and_noncommuting_rpy(self):
        for rpy in ([math.pi, 0, 0], [0, math.pi, 0], [0, 0, math.pi], [1.2, -0.7, 2.6]):
            matrix = transform([1, 2, 3], rpy)
            pose = matrix_pose(matrix)
            q = pose["rotation"]
            self.assertAlmostEqual(sum(value * value for value in q), 1)
            # Rotate ROS +Z directly by the quaternion and compare the matrix column.
            x, y, z, w = q
            z_axis = [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)]
            for i in range(3):
                self.assertAlmostEqual(z_axis[i], matrix[i][2])


class RuntimeVerifierTests(unittest.TestCase):
    def setUp(self):
        # A compact fixture with an intermediate frame ensures matching only TCP
        # cannot conceal an incorrect arm origin.
        poses = [{"name": name, "position": [0.0, 0.0, 0.0], "rotation": [0.0, 0.0, 0.0, 1.0]}
                 for name in ("ur_base_link", "ur_forearm_link", "tcp")]
        self.expected = {"models": [{"tool": "none", "goldenCases": [{"name": "zero", "links": poses}]}]}
        self.actual = runtime_copy(self.expected)

    def test_complete_matching_runtime_passes_and_counts_every_link(self):
        report = verify(self.expected, self.actual)
        self.assertTrue(report["passed"])
        self.assertEqual(report["verifiedLinkPoses"], 3)

    def test_identical_tcp_does_not_hide_an_incorrect_intermediate_link(self):
        self.actual["models"][0]["cases"][0]["links"][1]["position"][0] = 0.014
        report = verify(self.expected, self.actual)
        self.assertFalse(report["passed"])
        self.assertEqual(report["positionErrorM"], 0.014)
        self.assertIn("ur_forearm_link", report["failures"][0])

    def test_missing_extra_tools_cases_and_links_fail(self):
        for mutation in (lambda d: d["models"].clear(),
                         lambda d: d["models"][0]["cases"].clear(),
                         lambda d: d["models"][0]["cases"][0]["links"].pop(),
                         lambda d: d["models"][0]["cases"][0]["links"].append(
                             dict(name="old_tool_left_behind", position=[0, 0, 0], rotation=[0, 0, 0, 1]))):
            changed = copy.deepcopy(self.actual)
            mutation(changed)
            self.assertFalse(verify(self.expected, changed)["passed"])

    def test_quaternion_sign_does_not_fail_but_changed_orientation_does(self):
        pose = self.actual["models"][0]["cases"][0]["links"][0]
        pose["rotation"] = [0, 0, 0, -1]
        self.assertTrue(verify(self.expected, self.actual)["passed"])
        pose["rotation"] = [0, math.sin(0.1), 0, math.cos(0.1)]
        self.assertFalse(verify(self.expected, self.actual)["passed"])

    def test_invalid_and_nonunit_runtime_rotations_fail(self):
        pose = self.actual["models"][0]["cases"][0]["links"][0]
        for rotation in ([0, 0, 0, 0], [0, 0, 0, 2], [0, 0, math.nan, 1]):
            pose["rotation"] = rotation
            self.assertFalse(verify(self.expected, self.actual)["passed"])

    def test_duplicate_runtime_link_is_rejected(self):
        links = self.actual["models"][0]["cases"][0]["links"]
        links.append(copy.deepcopy(links[0]))
        with self.assertRaisesRegex(ValueError, "Duplicate link"):
            verify(self.expected, self.actual)


if __name__ == "__main__":
    unittest.main()
