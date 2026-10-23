#include <gtest/gtest.h>
#include "ur16e_mtc_gpio_stage/pick_place_targets.hpp"
#include <limits>
using namespace ur16e_mtc_gpio_stage;
namespace {
ur16e_interfaces::msg::PickPlacePoses contacts()
{
  ur16e_interfaces::msg::PickPlacePoses targets;
  for (auto *p : {&targets.pick_pose, &targets.place_pose})
  {
    p->header.frame_id = "ur_base_link";
    p->pose.position.x = 0.4;
    p->pose.position.z = 0.3;
    p->pose.orientation.y = std::sqrt(0.5); // +Z points +baseX
    p->pose.orientation.w = std::sqrt(0.5);
  }
  return targets;
}
TEST(PickPlaceContacts, ObjectCentreFollowsToolNormalNotBaseVertical)
{
  auto target = validatedContactTargets(contacts()).pick_pose;
  auto object = contactObjectCentre(target, 0.004);
  EXPECT_NEAR(object.pose.position.x, 0.402, 1e-12);
  EXPECT_NEAR(object.pose.position.z, 0.3, 1e-12);
  // Existing grasp offset -(approach+h/2) then +approach ends at target.
  EXPECT_NEAR(object.pose.position.x - 0.102 + 0.1, target.pose.position.x, 1e-12);
  target.pose.orientation = {};
  target.pose.orientation.x = 1; target.pose.orientation.w = 0; // downward tool
  object = contactObjectCentre(target, 0.004);
  EXPECT_NEAR(object.pose.position.z, target.pose.position.z - 0.002, 1e-12);
}
TEST(PickPlaceContacts, FreeTwistPreservesContactAndNormal)
{
  auto target = validatedContactTargets(contacts()).place_pose;
  const auto centre = contactObjectCentre(target, 0.004);
  for (int i = 0; i < 12; ++i)
  {
    const auto sample = contactYawSample(target, i*2*3.141592653589793/12);
    EXPECT_EQ(sample.pose.position, target.pose.position);
    const auto sample_centre = contactObjectCentre(sample, 0.004);
    EXPECT_NEAR(sample_centre.pose.position.x, centre.pose.position.x, 1e-12);
    EXPECT_NEAR(sample_centre.pose.position.y, centre.pose.position.y, 1e-12);
    EXPECT_NEAR(sample_centre.pose.position.z, centre.pose.position.z, 1e-12);
  }
}
TEST(PickPlaceContacts, BothTargetsValidateBeforeAnySceneMutation)
{
  auto target = contacts();
  target.place_pose.header.frame_id = "world";
  EXPECT_THROW(validatedContactTargets(target), std::invalid_argument);
  target = contacts(); target.place_pose.pose.orientation = {}; target.place_pose.pose.orientation.w = 0;
  EXPECT_THROW(validatedContactTargets(target), std::invalid_argument);
  target = contacts(); target.place_pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(validatedContactTargets(target), std::invalid_argument);
  EXPECT_THROW(contactObjectCentre(contacts().pick_pose, -0.004), std::invalid_argument);
}
TEST(PickPlaceContacts, NormalizeWithoutMovingContact)
{
  auto target = contacts(); target.pick_pose.header.stamp.sec = 42;
  target.pick_pose.pose.orientation.y *= 2; target.pick_pose.pose.orientation.w *= 2;
  const auto result = validatedContactTargets(target);
  EXPECT_EQ(result.pick_pose.pose.position, target.pick_pose.pose.position);
  EXPECT_EQ(result.pick_pose.header.stamp.sec, 0);
  EXPECT_NEAR(result.pick_pose.pose.orientation.y, std::sqrt(0.5), 1e-12);
}
} // namespace
