#include <gtest/gtest.h>

#include "ur16e_mtc_gpio_stage/plan_validation.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace
{
using Scene = moveit_msgs::msg::PlanningScene;
using ur16e_mtc_gpio_stage::validateJointStart;
using ur16e_mtc_gpio_stage::validatePlanStart;
constexpr double TOLERANCE = 0.01;

moveit_msgs::msg::CollisionObject box(const std::string &id, const std::string &frame)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = id;
  object.header.frame_id = frame;
  object.pose.orientation.w = 1.0;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = {0.1, 0.2, 0.3};
  object.primitive_poses.resize(1);
  object.primitive_poses[0].orientation.w = 1.0;
  object.subframe_names = {"first", "second"};
  object.subframe_poses.resize(2);
  object.subframe_poses[0].orientation.w = 1.0;
  object.subframe_poses[1].orientation.w = 1.0;
  object.subframe_poses[1].position.z = 0.2;
  return object;
}

Scene snapshot()
{
  Scene scene;
  scene.name = "planned";
  scene.robot_model_name = "ur16e";
  scene.robot_state.joint_state.name = {"shoulder", "elbow"};
  scene.robot_state.joint_state.position = {0.1, -0.2};
  scene.world.collision_objects = {box("tile", "fixture"), box("fixture", "base")};
  for (const auto &id : {"held_a", "held_b"})
  {
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = "tcp";
    attached.object = box(id, "tcp");
    attached.touch_links = {"tcp", "wrist"};
    attached.detach_posture.joint_names = {"gripper"};
    scene.robot_state.attached_collision_objects.push_back(attached);
  }
  auto &matrix = scene.allowed_collision_matrix;
  matrix.entry_names = {"tile", "tcp"};
  matrix.entry_values.resize(2);
  matrix.entry_values[0].enabled = {false, true};
  matrix.entry_values[1].enabled = {true, true};
  matrix.default_entry_names = {"tile", "tcp"};
  matrix.default_entry_values = {false, true};
  for (const auto &name : {"fixture", "camera"})
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = name;
    transform.child_frame_id = "base";
    transform.transform.rotation.w = 1.0;
    scene.fixed_frame_transforms.push_back(transform);
    moveit_msgs::msg::LinkPadding padding;
    padding.link_name = name;
    padding.padding = 0.01;
    scene.link_padding.push_back(padding);
    moveit_msgs::msg::LinkScale scale;
    scale.link_name = name;
    scale.scale = 1.0;
    scene.link_scale.push_back(scale);
  }
  scene.world.octomap.header.frame_id = "base";
  scene.world.octomap.origin.orientation.w = 1.0;
  scene.world.octomap.octomap.header.frame_id = "base";
  scene.world.octomap.octomap.id = "OcTree";
  scene.world.octomap.octomap.binary = true;
  scene.world.octomap.octomap.resolution = 0.05;
  scene.world.octomap.octomap.data = {1, 2};
  return scene;
}

TEST(PlanValidation, JointComparisonUsesNamesToleranceAndBoundedAngles)
{
  const auto expected = snapshot().robot_state.joint_state;
  auto current = expected;
  current.name = {"extra", "elbow", "shoulder"};
  current.position = {4.0, -0.195, 0.105};
  current.header.stamp.sec = 200;
  current.velocity = {3.0};
  current.effort = {5.0};
  EXPECT_TRUE(validateJointStart(expected, current, TOLERANCE).empty());
  current.position[2] = 0.12;
  EXPECT_FALSE(validateJointStart(expected, current, TOLERANCE).empty());
  current.position[2] = 0.1 + 6.283185307179586;
  EXPECT_FALSE(validateJointStart(expected, current, TOLERANCE).empty());
}

TEST(PlanValidation, JointComparisonRejectsIncompleteAmbiguousAndNonfiniteFeedback)
{
  const auto expected = snapshot().robot_state.joint_state;
  std::vector<sensor_msgs::msg::JointState> invalid;
  invalid.emplace_back();
  auto changed = expected;
  changed.name[1] = "shoulder";
  invalid.push_back(changed);
  changed = expected;
  changed.position.pop_back();
  invalid.push_back(changed);
  changed = expected;
  changed.name[1] = "unknown";
  invalid.push_back(changed);
  changed = expected;
  changed.position[0] = std::numeric_limits<double>::quiet_NaN();
  invalid.push_back(changed);
  changed.position[0] = std::numeric_limits<double>::infinity();
  invalid.push_back(changed);
  for (const auto &state : invalid)
    EXPECT_FALSE(validateJointStart(expected, state, TOLERANCE).empty());
  EXPECT_FALSE(validateJointStart({}, expected, TOLERANCE).empty());
  EXPECT_FALSE(validateJointStart(expected, expected, -0.01).empty());
  EXPECT_FALSE(validateJointStart(expected, expected, std::numeric_limits<double>::quiet_NaN()).empty());
}

TEST(PlanValidation, EquivalentOrderingAndVolatileMetadataDoNotInvalidatePlan)
{
  const auto expected = snapshot();
  auto current = expected;
  current.name = "live scene name";
  current.object_colors.resize(1);
  current.object_colors[0].id = "tile";
  current.object_colors[0].color.r = 1.0;
  auto &state = current.robot_state;
  std::reverse(state.joint_state.name.begin(), state.joint_state.name.end());
  state.joint_state.position = {-0.195, 0.105};
  state.joint_state.header.stamp.sec = 123;
  state.joint_state.velocity = {0.2, 0.3};
  state.multi_dof_joint_state.header.stamp.sec = 123;
  std::reverse(current.world.collision_objects.begin(), current.world.collision_objects.end());
  const auto reorder_object = [](auto &object) {
    object.header.stamp.sec = 123;
    object.type.key = "database metadata";
    std::reverse(object.subframe_names.begin(), object.subframe_names.end());
    std::reverse(object.subframe_poses.begin(), object.subframe_poses.end());
  };
  for (auto &object : current.world.collision_objects)
    reorder_object(object);
  std::reverse(state.attached_collision_objects.begin(), state.attached_collision_objects.end());
  for (auto &attachment : state.attached_collision_objects)
  {
    reorder_object(attachment.object);
    std::reverse(attachment.touch_links.begin(), attachment.touch_links.end());
    attachment.detach_posture.header.stamp.sec = 123;
  }
  auto &matrix = current.allowed_collision_matrix;
  std::reverse(matrix.entry_names.begin(), matrix.entry_names.end());
  std::reverse(matrix.entry_values.begin(), matrix.entry_values.end());
  for (auto &row : matrix.entry_values)
    std::reverse(row.enabled.begin(), row.enabled.end());
  std::reverse(matrix.default_entry_names.begin(), matrix.default_entry_names.end());
  std::reverse(matrix.default_entry_values.begin(), matrix.default_entry_values.end());
  std::reverse(current.fixed_frame_transforms.begin(), current.fixed_frame_transforms.end());
  for (auto &transform : current.fixed_frame_transforms)
    transform.header.stamp.sec = 123;
  std::reverse(current.link_padding.begin(), current.link_padding.end());
  std::reverse(current.link_scale.begin(), current.link_scale.end());
  current.world.octomap.header.stamp.sec = 123;
  current.world.octomap.octomap.header.stamp.sec = 123;
  const auto before = current;
  EXPECT_TRUE(validatePlanStart(expected, current, TOLERANCE).empty());
  EXPECT_EQ(current, before);
  EXPECT_EQ(expected, snapshot());
}

TEST(PlanValidation, MoveItTransformsShareTargetAndUseUniqueSources)
{
  auto planned = snapshot();
  geometry_msgs::msg::TransformStamped identity;
  identity.header.frame_id = identity.child_frame_id = "base";
  identity.transform.rotation.w = 1.0;
  planned.fixed_frame_transforms.push_back(identity);
  auto measured = planned;
  std::reverse(measured.fixed_frame_transforms.begin(), measured.fixed_frame_transforms.end());
  for (auto &entry : measured.fixed_frame_transforms) entry.header.stamp.sec = 9;
  EXPECT_TRUE(validatePlanStart(planned, measured, TOLERANCE).empty());
  measured.fixed_frame_transforms[0].child_frame_id.clear();
  EXPECT_FALSE(validatePlanStart(planned, measured, TOLERANCE).empty());
}

TEST(PlanValidation, UnreferencedControllerFeedbackDoesNotChangeCollisionScene)
{
  auto planned = snapshot();
  geometry_msgs::msg::TransformStamped controller;
  controller.header.frame_id = "ur_tool0_controller";
  controller.child_frame_id = "base";
  controller.transform.rotation.w = 1.0;
  planned.fixed_frame_transforms.push_back(controller);
  auto measured = planned;
  measured.fixed_frame_transforms.back().transform.translation.x = 0.00001;
  measured.fixed_frame_transforms.back().transform.rotation.z = 0.00001;
  EXPECT_TRUE(validatePlanStart(planned, measured, TOLERANCE).empty());
  // Once geometry references that frame, the exact transform is relevant.
  planned.world.collision_objects[0].header.frame_id = "ur_tool0_controller";
  measured.world.collision_objects[0].header.frame_id = "ur_tool0_controller";
  EXPECT_FALSE(validatePlanStart(planned, measured, TOLERANCE).empty());
}

TEST(PlanValidation, ReferencedTransformNamesTargetAndRotationRemainProtected)
{
  const auto planned = snapshot();
  for (int change = 0; change < 4; ++change)
  {
    auto measured = planned;
    auto &transform = measured.fixed_frame_transforms[0]; // referenced fixture
    if (change == 0) transform.header.frame_id = "another_source";
    if (change == 1) transform.child_frame_id = "another_target";
    if (change == 2) transform.transform.rotation.x = 0.1;
    if (change == 3) measured.fixed_frame_transforms.erase(measured.fixed_frame_transforms.begin());
    EXPECT_FALSE(validatePlanStart(planned, measured, TOLERANCE).empty());
  }
}

TEST(PlanValidation, AttachmentAndNonemptyOctomapFramesRemainProtected)
{
  for (bool attachment_frame : {false, true})
  {
    auto planned = snapshot();
    geometry_msgs::msg::TransformStamped external;
    external.header.frame_id = "external_collision_frame";
    external.child_frame_id = "base";
    external.transform.rotation.w = 1.0;
    planned.fixed_frame_transforms.push_back(external);
    if (attachment_frame)
      planned.robot_state.attached_collision_objects[0].object.header.frame_id = external.header.frame_id;
    else
      planned.world.octomap.header.frame_id = external.header.frame_id;
    auto measured = planned;
    measured.fixed_frame_transforms.back().transform.translation.y += 0.01;
    EXPECT_FALSE(validatePlanStart(planned, measured, TOLERANCE).empty());
    if (!attachment_frame)
    {
      // An empty map's metadata alone must not make an unused live TF relevant.
      planned.world.octomap.octomap.data.clear();
      measured.world.octomap.octomap.data.clear();
      EXPECT_TRUE(validatePlanStart(planned, measured, TOLERANCE).empty());
    }
  }
}

using Mutation = std::pair<std::string, std::function<void(Scene &)>>;

TEST(PlanValidation, CollisionRelevantChangesRejectThePlan)
{
  const auto expected = snapshot();
  const std::vector<Mutation> mutations{
      {"robot position", [](auto &s) { s.robot_state.joint_state.position[0] += 0.1; }},
      {"model", [](auto &s) { s.robot_model_name = "another_robot"; }},
      {"added world object", [](auto &s) { s.world.collision_objects.push_back(box("new", "base")); }},
      {"removed world object", [](auto &s) { s.world.collision_objects.pop_back(); }},
      {"object pose", [](auto &s) { s.world.collision_objects[0].pose.position.x += 0.01; }},
      {"object frame", [](auto &s) { s.world.collision_objects[0].header.frame_id = "camera"; }},
      {"shape", [](auto &s) { s.world.collision_objects[0].primitives[0].dimensions[0] += 0.01; }},
      {"shape pose", [](auto &s) { s.world.collision_objects[0].primitive_poses[0].position.z += 0.01; }},
      {"subframe", [](auto &s) { s.world.collision_objects[0].subframe_poses[0].position.z += 0.01; }},
      {"attachment removed", [](auto &s) { s.robot_state.attached_collision_objects.pop_back(); }},
      {"attachment link", [](auto &s) { s.robot_state.attached_collision_objects[0].link_name = "wrist"; }},
      {"attachment pose", [](auto &s) { s.robot_state.attached_collision_objects[0].object.pose.position.x += 0.01; }},
      {"touch links", [](auto &s) { s.robot_state.attached_collision_objects[0].touch_links.pop_back(); }},
      {"collision permission", [](auto &s) {
         s.allowed_collision_matrix.entry_values[0].enabled[1] = false;
         s.allowed_collision_matrix.entry_values[1].enabled[0] = false;
       }},
      {"default collision permission", [](auto &s) { s.allowed_collision_matrix.default_entry_values[0] = true; }},
      {"fixed transform", [](auto &s) { s.fixed_frame_transforms[0].transform.translation.x += 0.01; }},
      {"padding", [](auto &s) { s.link_padding[0].padding += 0.01; }},
      {"scale", [](auto &s) { s.link_scale[0].scale += 0.01; }},
      {"octomap cells", [](auto &s) { s.world.octomap.octomap.data[0] = 3; }},
      {"octomap pose", [](auto &s) { s.world.octomap.origin.position.x += 0.01; }},
  };
  for (const auto &[name, mutate] : mutations)
  {
    SCOPED_TRACE(name);
    auto current = expected;
    mutate(current);
    EXPECT_FALSE(validatePlanStart(expected, current, TOLERANCE).empty());
  }
}

TEST(PlanValidation, PartialMalformedOrUnsupportedScenesRejectThePlan)
{
  const auto expected = snapshot();
  const std::vector<Mutation> mutations{
      {"scene diff", [](auto &s) { s.is_diff = true; }},
      {"state diff", [](auto &s) { s.robot_state.is_diff = true; }},
      {"missing model", [](auto &s) { s.robot_model_name.clear(); }},
      {"extra scene joint", [](auto &s) {
         s.robot_state.joint_state.name.push_back("extra");
         s.robot_state.joint_state.position.push_back(0.0);
       }},
      {"multi-DOF state", [](auto &s) { s.robot_state.multi_dof_joint_state.joint_names = {"base"}; }},
      {"duplicate object", [](auto &s) { s.world.collision_objects.push_back(s.world.collision_objects[0]); }},
      {"missing shape pose", [](auto &s) { s.world.collision_objects[0].primitive_poses.clear(); }},
      {"object delta", [](auto &s) { s.world.collision_objects[0].operation = 1; }},
      {"duplicate attachment", [](auto &s) {
         s.robot_state.attached_collision_objects.push_back(s.robot_state.attached_collision_objects[0]);
       }},
      {"duplicate transform", [](auto &s) { s.fixed_frame_transforms.push_back(s.fixed_frame_transforms[0]); }},
      {"empty transform source", [](auto &s) { s.fixed_frame_transforms[0].header.frame_id.clear(); }},
      {"empty transform target", [](auto &s) { s.fixed_frame_transforms[0].child_frame_id.clear(); }},
      {"missing matrix row", [](auto &s) { s.allowed_collision_matrix.entry_values.pop_back(); }},
      {"short matrix row", [](auto &s) { s.allowed_collision_matrix.entry_values[1].enabled.pop_back(); }},
      {"asymmetric matrix", [](auto &s) { s.allowed_collision_matrix.entry_values[0].enabled[1] = false; }},
      {"duplicate matrix name", [](auto &s) { s.allowed_collision_matrix.entry_names[1] = "tile"; }},
      {"missing default value", [](auto &s) { s.allowed_collision_matrix.default_entry_values.pop_back(); }},
  };
  for (const auto &[name, mutate] : mutations)
  {
    SCOPED_TRACE(name);
    auto invalid = expected;
    mutate(invalid);
    EXPECT_FALSE(validatePlanStart(expected, invalid, TOLERANCE).empty());
    EXPECT_FALSE(validatePlanStart(invalid, expected, TOLERANCE).empty());
  }
}
} // namespace
