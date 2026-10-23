"""Prepare the user-authorized ROS patch without modifying the external checkout."""
from pathlib import Path
import difflib
import json

SOURCE=Path('C:/Users/aca/source/UR16UnityControl')
OUT=Path(__file__).parent/'BackendWorkflowPatch'
files={}
def read(path): return files.get(path,(SOURCE/path).read_text(encoding='utf-8-sig'))
def put(path,text): files[path]=text
def replace(path,old,new):
    text=read(path)
    assert text.count(old)==1,(path,old[:100],text.count(old))
    put(path,text.replace(old,new))
P='ros_ws/src/ur16e_mtc_gpio_stage/'
V=P+'include/ur16e_mtc_gpio_stage/plan_validation.hpp'
replace(V,'#include <map>','#include <map>\n#include <set>')
replace(V,'''  for (auto &transform : scene.fixed_frame_transforms)
    transform.header.stamp = {};
  if (!sortUnique(scene.fixed_frame_transforms, [](const auto &entry) { return entry.child_frame_id; }))
    return "invalid fixed frame names";''','''  // MoveIt serializes source in header.frame_id and the common planning
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
  }), transforms.end());''')

T=P+'test/test_plan_validation.cpp'
replace(T,'scene.world.collision_objects = {box("tile", "base"), box("fixture", "base")};',
        'scene.world.collision_objects = {box("tile", "fixture"), box("fixture", "base")};')
replace(T,'transform.header.frame_id = "base";\n    transform.child_frame_id = name;',
        'transform.header.frame_id = name;\n    transform.child_frame_id = "base";')
replace(T,'using Mutation = std::pair<std::string, std::function<void(Scene &)>>;',r'''TEST(PlanValidation, MoveItTransformsShareTargetAndUseUniqueSources)
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

using Mutation = std::pair<std::string, std::function<void(Scene &)>>;''')

put(P+'include/ur16e_mtc_gpio_stage/pick_place_targets.hpp',r'''#pragma once

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
''')

S=P+'src/test_task.cpp'
replace(S,'#include "ur16e_mtc_gpio_stage/tcp_waypoints.hpp"',
        '#include "ur16e_mtc_gpio_stage/tcp_waypoints.hpp"\n#include "ur16e_mtc_gpio_stage/pick_place_targets.hpp"')
replace(S,'#include <stdexcept>','#include <stdexcept>\n#include <optional>')
replace(S,'std::string ik_frame, bool tcp_pose_planning_enabled)',
        'std::string ik_frame, bool tcp_pose_planning_enabled, bool pick_place_tcp_planning_enabled)')
replace(S,'"pick_place_poses_target", 10, std::bind(&MTCTestTask::createTask, this, _1), command_options);',
'''"pick_place_poses_target", 10, std::bind(&MTCTestTask::createTask, this, _1), command_options);
        if (pick_place_tcp_planning_enabled)
            pick_place_tcp_sub_ = node_->create_subscription<ur16e_interfaces::msg::PickPlacePoses>(
                "pick_place_tcp_target", 10, std::bind(&MTCTestTask::createContactTask, this, _1), command_options);''')
replace(S,'rclcpp::Subscription<ur16e_interfaces::msg::PickPlacePoses>::SharedPtr pick_place_sub_;',
'''rclcpp::Subscription<ur16e_interfaces::msg::PickPlacePoses>::SharedPtr pick_place_sub_;
    rclcpp::Subscription<ur16e_interfaces::msg::PickPlacePoses>::SharedPtr pick_place_tcp_sub_;
    std::optional<moveit_msgs::msg::CollisionObject> owned_preview_object_;''')
replace(S,'void applyBox(const ur16e_interfaces::msg::Box &box_msg)',
        'void applyBox(const ur16e_interfaces::msg::Box &box_msg, bool task_preview = false)')
replace(S,'object.pose = box_msg.position.pose;\n        object.operation = object.ADD;',
'''object.pose = box_msg.position.pose;
        object.primitive_poses.resize(1);
        object.primitive_poses[0].orientation.w = 1.0;
        object.operation = object.ADD;''')
replace(S,'''        if (!callSceneService<ApplyPlanningScene>(scene_apply_client_, request)->success)
            throw std::runtime_error("MoveIt rejected the planning-scene box update");
    }

    void doTask''','''        if (!callSceneService<ApplyPlanningScene>(scene_apply_client_, request)->success)
            throw std::runtime_error("MoveIt rejected the planning-scene box update");
        owned_preview_object_.reset();
        if (task_preview)
        {
            auto query = std::make_shared<GetPlanningScene::Request>();
            query->components.components = moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
            const auto response = callSceneService<GetPlanningScene>(scene_query_client_, query);
            for (const auto &entry : response->scene.world.collision_objects)
                if (entry.id == "object") owned_preview_object_ = entry;
        }
    }

    void removeUnexecutedPreviewObject()
    {
        if (!owned_preview_object_) return;
        auto query = std::make_shared<GetPlanningScene::Request>();
        query->components.components = moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
            moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
        const auto response = callSceneService<GetPlanningScene>(scene_query_client_, query);
        for (const auto &attached : response->scene.robot_state.attached_collision_objects)
            if (attached.object.id == "object") return; // held physical objects stay attached
        for (const auto &entry : response->scene.world.collision_objects)
        {
            if (entry.id != "object") continue;
            auto previous = *owned_preview_object_;
            auto current = entry;
            if (!ur16e_mtc_gpio_stage::plan_validation_detail::normalizeObject(previous) ||
                !ur16e_mtc_gpio_stage::plan_validation_detail::normalizeObject(current) || previous != current)
                throw std::runtime_error("Previous task preview object changed externally; inspect the scene before TCP planning");
            auto removal = std::make_shared<ApplyPlanningScene::Request>();
            removal->scene.is_diff = true;
            removal->scene.robot_state.is_diff = true;
            moveit_msgs::msg::CollisionObject object;
            object.id = "object";
            object.operation = object.REMOVE;
            removal->scene.world.collision_objects.push_back(object);
            if (!callSceneService<ApplyPlanningScene>(scene_apply_client_, removal)->success)
                throw std::runtime_error("MoveIt rejected removal of the unexecuted task preview");
            break;
        }
        owned_preview_object_.reset();
    }

    void doTask''')
replace(S,'const bool vacuum_task = plan->mode == TaskMode::VacuumPickPlace;',
'''const bool vacuum_task = plan->mode == TaskMode::VacuumPickPlace;
            // After any vacuum execution attempt, the object may be physical or
            // moved/attached. Never automatically remove it as a preview again.
            if (vacuum_task) owned_preview_object_.reset();''')
replace(S,'''    void createTask(ur16e_interfaces::msg::PickPlacePoses poses)
    {
        std::unique_lock''','''    void createTask(ur16e_interfaces::msg::PickPlacePoses poses)
    {
        createTaskImpl(std::move(poses), false);
    }

    void createContactTask(ur16e_interfaces::msg::PickPlacePoses poses)
    {
        createTaskImpl(std::move(poses), true);
    }

    void createTaskImpl(ur16e_interfaces::msg::PickPlacePoses poses, bool tool_contacts)
    {
        std::unique_lock''')
replace(S,'            buildTask(poses);',
'''            if (tool_contacts)
                poses = ur16e_mtc_gpio_stage::validatedContactTargets(std::move(poses));
            buildTask(poses, tool_contacts);''')
replace(S,'            task_.stages()->setName("Exact TCP A then B");',
'''            removeUnexecutedPreviewObject();
            task_.stages()->setName("Exact TCP A then B");''')
replace(S,'void buildTask(const ur16e_interfaces::msg::PickPlacePoses &poses)',
        'void buildTask(const ur16e_interfaces::msg::PickPlacePoses &poses, bool tool_contacts)')
replace(S,'''        PoseStamped tile_pose = poses.pick_pose;
        tile_pose.pose.position.z -= tile_height / 2; // only valid if reference frame has same +z as tile 
        ur16e_interfaces::msg::Box b = createBoxMessage(tile_pose);
        applyBox(b);''','''        PoseStamped tile_pose = poses.pick_pose;
        if (tool_contacts)
            tile_pose = ur16e_mtc_gpio_stage::contactObjectCentre(tile_pose, tile_height);
        else
            tile_pose.pose.position.z -= tile_height / 2; // legacy horizontal surface convention
        ur16e_interfaces::msg::Box b = createBoxMessage(tile_pose);
        applyBox(b, true);''')
replace(S,'task_.stages()->setName("UR GPIO + Motion");',
        'task_.stages()->setName(tool_contacts ? "Tool contact pick/place" : "UR GPIO + Motion");')
replace(S,'''                    RCLCPP_DEBUG(LOGGER, "Creating GeneratePlacePose stage 'generate place pose'");''','''                    if (tool_contacts)
                    {
                        auto alternatives = std::make_unique<mtc::Alternatives>("place tool twist alternatives");
                        task_.properties().exposeTo(alternatives->properties(), {"group", "ik_frame"});
                        alternatives->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});
                        for (int index = 0; index < 12; ++index)
                        {
                            auto generator = std::make_unique<mtc::stages::GeneratePose>("place contact " + std::to_string(index));
                            generator->properties().configureInitFrom(mtc::Stage::PARENT);
                            generator->setPose(ur16e_mtc_gpio_stage::contactYawSample(
                                poses.place_pose, 2.0 * M_PI * index / 12));
                            generator->setMonitoredStage(attach_object_stage);
                            auto ik = std::make_unique<mtc::stages::ComputeIK>("place contact IK " + std::to_string(index), std::move(generator));
                            ik->setMaxIKSolutions(2);
                            ik->setMinSolutionDistance(0.2);
                            ik->setIKFrame(ik_frame_);
                            ik->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                            ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
                            alternatives->insert(std::move(ik));
                        }
                        place->insert(std::move(alternatives));
                    }
                    else
                    {
                    RCLCPP_DEBUG(LOGGER, "Creating GeneratePlacePose stage 'generate place pose'");''')
replace(S,'''                    RCLCPP_DEBUG(LOGGER, "ComputeIK wrapper added to place container");
                }''','''                    RCLCPP_DEBUG(LOGGER, "ComputeIK wrapper added to place container");
                    }
                }''')
replace(S,'        MTCTestTask testTask(node, tool, ik_frame, tcp_planning);',
'''        const bool contact_planning = ur16e_mtc_gpio_stage::readOnlyStartupBool(
            node, "pick_place_tcp_planning_enabled", true);
        MTCTestTask testTask(node, tool, ik_frame, tcp_planning, contact_planning);''')

put(P+'test/test_pick_place_targets.cpp',r'''#include <gtest/gtest.h>
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
  target.pose.orientation.x = 1; // downward tool
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
  target = contacts(); target.place_pose.pose.orientation = {};
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
''')
C=P+'CMakeLists.txt'
replace(C,'  ament_add_gtest(test_startup_parameter test/test_startup_parameter.cpp)',
'''  ament_add_gtest(test_pick_place_targets test/test_pick_place_targets.cpp)
  target_compile_features(test_pick_place_targets PRIVATE cxx_std_17)
  target_include_directories(test_pick_place_targets PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_pick_place_targets geometry_msgs ur16e_interfaces)

  ament_add_gtest(test_startup_parameter test/test_startup_parameter.cpp)''')

patch=[]
for path,text in files.items():
    destination=OUT/path
    destination.parent.mkdir(parents=True,exist_ok=True)
    destination.write_text(text,encoding='utf-8',newline='\n')
    old=(SOURCE/path).read_text(encoding='utf-8-sig') if (SOURCE/path).exists() else ''
    patch.extend(difflib.unified_diff(old.splitlines(keepends=True),text.splitlines(keepends=True),
        fromfile='a/'+path if old else '/dev/null',tofile='b/'+path))
OUT.mkdir(parents=True,exist_ok=True)
(OUT/'backend-workflow.patch').write_text(''.join(patch),encoding='utf-8')
(OUT/'manifest.json').write_text(json.dumps(list(files),indent=2),encoding='utf-8')
print(json.dumps({'files':len(files),'staging':str(OUT)},indent=2))
