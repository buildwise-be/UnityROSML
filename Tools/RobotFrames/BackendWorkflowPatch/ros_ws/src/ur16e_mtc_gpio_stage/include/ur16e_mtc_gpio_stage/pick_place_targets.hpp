#pragma once

#include <ur16e_interfaces/msg/pick_place_poses.hpp>
#include <cmath>
#include <stdexcept>

namespace ur16e_mtc_gpio_stage
{
inline ur16e_interfaces::msg::PickPlacePoses validatedContactTargets(
    ur16e_interfaces::msg::PickPlacePoses targets)
{
  for (auto *target : {&targets.pick_pose, &targets.place_pose})
  {
    if (target->header.frame_id != "ur_base_link")
      throw std::invalid_argument("Pick/place TCP contacts must be expressed in ur_base_link");
    const auto &p = target->pose.position;
    auto &q = target->pose.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
      throw std::invalid_argument("Pick/place TCP contacts require finite poses");
    const double length = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (!std::isfinite(length) || length < 1e-12)
      throw std::invalid_argument("Pick/place TCP contacts require nonzero quaternions");
    q.x /= length; q.y /= length; q.z /= length; q.w /= length;
    target->header.stamp = {}; // static base-relative contact, never historical TF
  }
  return targets;
}

// The virtual object's centre lies beyond the foam contact along outward tool
// +Z. This rotates the offset with the tool, including tilted/vertical picks.
inline geometry_msgs::msg::PoseStamped contactObjectCentre(
    geometry_msgs::msg::PoseStamped contact, double height)
{
  if (!std::isfinite(height) || height <= 0.0)
    throw std::invalid_argument("Object thickness must be finite and positive");
  const auto &q = contact.pose.orientation;
  contact.pose.position.x += height * (q.x*q.z + q.w*q.y);
  contact.pose.position.y += height * (q.y*q.z - q.w*q.x);
  contact.pose.position.z += height * (0.5 - q.x*q.x - q.y*q.y);
  return contact;
}

// Changing this local twist preserves TCP position and the requested +Z normal.
inline geometry_msgs::msg::PoseStamped contactYawSample(
    geometry_msgs::msg::PoseStamped contact, double yaw)
{
  auto &q = contact.pose.orientation;
  const auto original = q;
  const double s = std::sin(yaw/2), c = std::cos(yaw/2);
  q.x = original.x*c + original.y*s;
  q.y = original.y*c - original.x*s;
  q.z = original.z*c + original.w*s;
  q.w = original.w*c - original.z*s;
  return contact;
}
} // namespace ur16e_mtc_gpio_stage
