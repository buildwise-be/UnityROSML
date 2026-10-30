#pragma once

#include <moveit_msgs/msg/planning_scene.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <utility>

namespace ur16e_mtc_gpio_stage
{
// UR arm joints are bounded revolute joints: deliberately do not wrap a delta
// modulo 2*pi. A different winding is a different trajectory start position.
inline std::string validateJointStart(const sensor_msgs::msg::JointState &expected,
                                      const sensor_msgs::msg::JointState &current,
                                      double tolerance)
{
  if (!std::isfinite(tolerance) || tolerance < 0.0)
    return "invalid joint tolerance";
  if (expected.name.empty())
    return "planned joint state is empty";

  std::map<std::string, double> planned, measured;
  const auto collect = [](const auto &state, auto &positions) {
    if (state.name.size() != state.position.size())
      return false;
    for (std::size_t i = 0; i < state.name.size(); ++i)
      if (state.name[i].empty() || !std::isfinite(state.position[i]) ||
          !positions.emplace(state.name[i], state.position[i]).second)
        return false;
    return true;
  };
  if (!collect(expected, planned))
    return "planned joint state is malformed";
  if (!collect(current, measured))
    return "current joint state is malformed";
  for (const auto &[name, position] : planned)
  {
    const auto found = measured.find(name);
    if (found == measured.end())
      return "current joint state is missing " + name;
    if (std::abs(position - found->second) > tolerance)
      return "start position changed for " + name;
  }
  return {};
}

namespace plan_validation_detail
{
template <class Entries, class Key>
bool sortUnique(Entries &entries, Key key)
{
  std::sort(entries.begin(), entries.end(), [&](const auto &a, const auto &b) { return key(a) < key(b); });
  for (std::size_t i = 0; i < entries.size(); ++i)
    if (key(entries[i]).empty() || (i > 0 && key(entries[i - 1]) == key(entries[i])))
      return false;
  return true;
}

template <class Names, class Values>
bool sortNamedValues(Names &names, Values &values)
{
  if (names.size() != values.size())
    return false;
  std::map<std::string, typename Values::value_type> ordered;
  for (std::size_t i = 0; i < names.size(); ++i)
    if (names[i].empty() || !ordered.emplace(names[i], values[i]).second)
      return false;
  names.clear();
  values.clear();
  for (const auto &[name, value] : ordered)
  {
    names.push_back(name);
    values.push_back(value);
  }
  return true;
}

inline bool normalizeMatrix(moveit_msgs::msg::AllowedCollisionMatrix &matrix)
{
  const auto source = matrix;
  const auto size = source.entry_names.size();
  if (source.entry_values.size() != size ||
      !sortNamedValues(matrix.default_entry_names, matrix.default_entry_values))
    return false;
  std::map<std::string, std::size_t> indices;
  for (std::size_t i = 0; i < size; ++i)
  {
    if (source.entry_names[i].empty() || source.entry_values[i].enabled.size() != size ||
        !indices.emplace(source.entry_names[i], i).second)
      return false;
  }
  matrix.entry_names.clear();
  std::size_t row = 0;
  for (const auto &[name, original_row] : indices)
  {
    matrix.entry_names.push_back(name);
    std::size_t column = 0;
    for (const auto &[unused_name, original_column] : indices)
    {
      (void)unused_name;
      const auto value = source.entry_values[original_row].enabled[original_column];
      if (value != source.entry_values[original_column].enabled[original_row])
        return false;
      matrix.entry_values[row].enabled[column++] = value;
    }
    ++row;
  }
  return true;
}

inline bool normalizeObject(moveit_msgs::msg::CollisionObject &object)
{
  object.header.stamp = {};
  // Full and component-filtered MoveIt serialization populate this database
  // metadata differently for attachments; it does not describe collision shape.
  object.type = {};
  return !object.header.frame_id.empty() && object.operation == object.ADD &&
         object.primitives.size() == object.primitive_poses.size() &&
         object.meshes.size() == object.mesh_poses.size() &&
         object.planes.size() == object.plane_poses.size() &&
         sortNamedValues(object.subframe_names, object.subframe_poses);
}

// MoveIt round-trips world poses through rotation matrices. The resulting unit
// quaternion can differ in its last bits even when collision geometry did not
// move. Align only those representations, on the local comparison copy; all
// positions, shapes, frame names and other scene data remain exact.
inline bool alignQuaternionRoundoff(const geometry_msgs::msg::Quaternion &expected,
                                   geometry_msgs::msg::Quaternion &current)
{
  constexpr double epsilon = 1e-12;
  const auto valid_unit = [=](const auto &q) {
    return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
           std::abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) <= epsilon;
  };
  if (!valid_unit(expected) || !valid_unit(current))
    return false;
  const auto near = [=](double a, double b) { return std::abs(a - b) <= epsilon; };
  const bool same = near(expected.x, current.x) && near(expected.y, current.y) &&
                    near(expected.z, current.z) && near(expected.w, current.w);
  // q and -q encode the same rotation; apply the same bound to either sign.
  const bool opposite = near(expected.x, -current.x) && near(expected.y, -current.y) &&
                        near(expected.z, -current.z) && near(expected.w, -current.w);
  if (same || opposite)
    current = expected;
  return true;
}

template <class Poses>
bool alignPoseQuaternionRoundoff(const Poses &expected, Poses &current)
{
  if (expected.size() != current.size())
    return false;
  for (std::size_t i = 0; i < expected.size(); ++i)
    if (!alignQuaternionRoundoff(expected[i].orientation, current[i].orientation))
      return false;
  return true;
}

inline bool alignWorldQuaternionRoundoff(const moveit_msgs::msg::PlanningSceneWorld &expected,
                                        moveit_msgs::msg::PlanningSceneWorld &current)
{
  if (expected.collision_objects.size() != current.collision_objects.size())
    return false;
  for (std::size_t i = 0; i < expected.collision_objects.size(); ++i)
  {
    const auto &planned = expected.collision_objects[i];
    auto &measured = current.collision_objects[i];
    if (planned.id != measured.id ||
        !alignQuaternionRoundoff(planned.pose.orientation, measured.pose.orientation) ||
        !alignPoseQuaternionRoundoff(planned.primitive_poses, measured.primitive_poses) ||
        !alignPoseQuaternionRoundoff(planned.mesh_poses, measured.mesh_poses) ||
        !alignPoseQuaternionRoundoff(planned.plane_poses, measured.plane_poses) ||
        !alignPoseQuaternionRoundoff(planned.subframe_poses, measured.subframe_poses))
      return false;
  }
  return true;
}

inline std::string normalizeScene(moveit_msgs::msg::PlanningScene &scene)
{
  if (scene.is_diff || scene.robot_state.is_diff)
    return "a full planning scene is required";
  if (scene.robot_model_name.empty())
    return "robot model name is missing";
  const auto &multi_dof = scene.robot_state.multi_dof_joint_state;
  if (!multi_dof.joint_names.empty() || !multi_dof.transforms.empty() ||
      !multi_dof.twist.empty() || !multi_dof.wrench.empty())
    return "multi-DOF robot state is unsupported for this fixed-base arm";
  // Joint positions were checked separately; feedback speed/effort and all
  // message timestamps are not collision geometry. Preserve coordinate frames.
  scene.robot_state.joint_state = {};
  scene.robot_state.multi_dof_joint_state = {};
  scene.name.clear();
  scene.object_colors.clear();

  auto &objects = scene.world.collision_objects;
  if (!sortUnique(objects, [](const auto &entry) { return entry.id; }))
    return "invalid world object IDs";
  for (auto &object : objects)
    if (!normalizeObject(object))
      return "malformed world object " + object.id;

  auto &attachments = scene.robot_state.attached_collision_objects;
  if (!sortUnique(attachments, [](const auto &entry) { return entry.object.id; }))
    return "invalid attached object IDs";
  for (auto &attachment : attachments)
  {
    attachment.detach_posture.header.stamp = {};
    if (attachment.link_name.empty() || !normalizeObject(attachment.object) ||
        !sortUnique(attachment.touch_links, [](const auto &entry) { return entry; }))
      return "malformed attached object " + attachment.object.id;
  }

  scene.world.octomap.header.stamp = {};
  scene.world.octomap.octomap.header.stamp = {};
  // MoveIt serializes source in header.frame_id and the common planning
  // target in child_frame_id (unlike a normal TF tree's unique children).
  for (auto &transform : scene.fixed_frame_transforms)
  {
    transform.header.stamp = {};
    if (transform.child_frame_id.empty())
      return "invalid fixed frame names";
  }
  if (!sortUnique(scene.fixed_frame_transforms, [](const auto &entry) { return entry.header.frame_id; }))
    return "invalid fixed frame names";

  // World/attached geometry is already frozen in its serialized frame. An
  // unrelated live TF, e.g. the controller's measured TCP, can jitter with
  // encoders without changing collision geometry. Compare only transforms
  // that these actual scene components reference; keep their values exact.
  std::set<std::string> collision_frames;
  for (const auto &object : objects)
    collision_frames.insert(object.header.frame_id);
  for (const auto &attachment : attachments)
    collision_frames.insert(attachment.object.header.frame_id);
  if (!scene.world.octomap.octomap.data.empty())
  {
    collision_frames.insert(scene.world.octomap.header.frame_id);
    collision_frames.insert(scene.world.octomap.octomap.header.frame_id);
  }
  auto &transforms = scene.fixed_frame_transforms;
  transforms.erase(std::remove_if(transforms.begin(), transforms.end(), [&](const auto &transform) {
    return collision_frames.count(transform.header.frame_id) == 0;
  }), transforms.end());
  if (!sortUnique(scene.link_padding, [](const auto &entry) { return entry.link_name; }) ||
      !sortUnique(scene.link_scale, [](const auto &entry) { return entry.link_name; }))
    return "invalid collision padding or scaling names";
  if (!normalizeMatrix(scene.allowed_collision_matrix))
    return "malformed allowed collision matrix";
  return {};
}
} // namespace plan_validation_detail

// Compare full snapshots from the chosen MTC solution and GetPlanningScene.
// Geometry is exact after key ordering, timestamp removal, and bounded unit
// quaternion representation alignment for world object poses only.
// This is a point-in-time preflight, not a lock against later scene writers.
inline std::string validatePlanStart(const moveit_msgs::msg::PlanningScene &expected,
                                    const moveit_msgs::msg::PlanningScene &current,
                                    double tolerance)
{
  if (const auto error = validateJointStart(expected.robot_state.joint_state,
                                          current.robot_state.joint_state, tolerance); !error.empty())
    return error;
  if (expected.robot_state.joint_state.name.size() != current.robot_state.joint_state.name.size())
    return "planning scene joint set changed";
  auto planned = expected;
  auto measured = current;
  if (const auto error = plan_validation_detail::normalizeScene(planned); !error.empty())
    return "planned scene: " + error;
  if (const auto error = plan_validation_detail::normalizeScene(measured); !error.empty())
    return "current scene: " + error;
  if (planned.robot_model_name != measured.robot_model_name)
    return "robot model changed";
  if (!plan_validation_detail::alignWorldQuaternionRoundoff(planned.world, measured.world) ||
      planned.world != measured.world)
    return "world collision geometry or octomap changed";
  if (planned.robot_state.attached_collision_objects != measured.robot_state.attached_collision_objects)
    return "attached collision objects changed";
  if (planned.allowed_collision_matrix != measured.allowed_collision_matrix)
    return "allowed collision matrix changed";
  if (planned.fixed_frame_transforms != measured.fixed_frame_transforms)
    return "fixed frame transforms changed";
  if (planned.link_padding != measured.link_padding || planned.link_scale != measured.link_scale)
    return "collision padding or scaling changed";
  return planned == measured ? std::string{} : "planning scene changed";
}
} // namespace ur16e_mtc_gpio_stage
