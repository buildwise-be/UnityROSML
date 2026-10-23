#include <rclcpp/rclcpp.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/storage.h> // SubTrajectory, SolutionBase
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/container.h>

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>

#include "ur16e_mtc_gpio_stage/set_io_stage.hpp"
#include "ur16e_mtc_gpio_stage/custom_executor.hpp"
#include "ur16e_mtc_gpio_stage/plan_gate.hpp"
#include "ur16e_mtc_gpio_stage/plan_validation.hpp"
#include "ur16e_mtc_gpio_stage/tool_definition.hpp"
#include "ur16e_mtc_gpio_stage/tool_model_validation.hpp"
#include "ur16e_mtc_gpio_stage/startup_parameter.hpp"
#include "ur16e_mtc_gpio_stage/tcp_waypoints.hpp"
#include "ur16e_mtc_gpio_stage/pick_place_targets.hpp"
#include "ur16e_interfaces/msg/pick_place_poses.hpp"
#include "ur16e_interfaces/msg/box.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // tf2::fromMsg / toMsg for geometry_msgs
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.hpp> // Eigen conversions (recommended)

#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/robot_state/conversions.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <mutex>
#include <map>
#include <vector>
#include <algorithm>
#include <sstream>
#include <string>
#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <optional>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_test_task");
namespace mtc = moveit::task_constructor;
using namespace std::placeholders;
float tile_height = 0.004;
float tile_size = 0.1;
float approach_distance = 0.1;

using PoseStamped = geometry_msgs::msg::PoseStamped;

class MTCTestTask
{
public:
    MTCTestTask(const rclcpp::Node::SharedPtr &node, const std::string &tool, std::string ik_frame, bool tcp_pose_planning_enabled, bool pick_place_tcp_planning_enabled)
        : node_(node), tool_(ur16e_mtc_gpio_stage::resolveToolDefinition(tool)), ik_frame_(std::move(ik_frame)), tcp_pose_planning_enabled_(tcp_pose_planning_enabled)
    {
        start_joint_tolerance_ = node_->get_parameter_or<double>("execution_start_tolerance", 0.01);
        joint_state_max_age_ = node_->get_parameter_or<double>("execution_joint_state_max_age", 1.0);
        if (!std::isfinite(start_joint_tolerance_) || start_joint_tolerance_ <= 0.0 ||
            !std::isfinite(joint_state_max_age_) || joint_state_max_age_ <= 0.0)
            throw std::invalid_argument("Execution tolerance and joint-state maximum age must be finite and positive");
        startup_robot_description_ = node_->get_parameter_or<std::string>("robot_description", "");
        startup_robot_description_semantic_ = node_->get_parameter_or<std::string>("robot_description_semantic", "");
        if (startup_robot_description_.empty() || startup_robot_description_semantic_.empty())
            throw std::invalid_argument("Set robot_description and robot_description_semantic before startup");
        task_.loadRobotModel(node_);
        startup_model_ = task_.getRobotModel();
        if (!startup_model_)
            throw std::runtime_error("Could not load the startup robot model");
        const auto model_error = ur16e_mtc_gpio_stage::validateToolModel(tool_, *startup_model_, ik_frame_);
        if (!model_error.empty())
            throw std::invalid_argument("Mounted tool configuration is invalid: " + model_error);
        RCLCPP_INFO(LOGGER, "Mounted tool=%s, TCP=%s, vacuum pick/place=%s",
                    tool_.name.c_str(), tool_.tcp_link.c_str(), tool_.vacuum ? "available" : "unavailable");

        // Admit or reject commands when their callbacks run, rather than queueing
        // them behind a long planning/execution callback in a mutually-exclusive group.
        command_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions command_options;
        command_options.callback_group = command_cb_group_;
        scene_client_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        scene_query_client_ = node_->create_client<GetPlanningScene>(
            "get_planning_scene", rmw_qos_profile_services_default, scene_client_cb_group_);
        scene_apply_client_ = node_->create_client<ApplyPlanningScene>(
            "apply_planning_scene", rmw_qos_profile_services_default, scene_client_cb_group_);

        scene_setup_sub_ = node_->create_subscription<ur16e_interfaces::msg::Box>(
            "scene_setup_box", 10, std::bind(&MTCTestTask::setupPlanningScene, this, _1), command_options);

        pick_place_sub_ = node_->create_subscription<ur16e_interfaces::msg::PickPlacePoses>(
            "pick_place_poses_target", 10, std::bind(&MTCTestTask::createTask, this, _1), command_options);
        if (pick_place_tcp_planning_enabled)
            pick_place_tcp_sub_ = node_->create_subscription<ur16e_interfaces::msg::PickPlacePoses>(
                "pick_place_tcp_target", 10, std::bind(&MTCTestTask::createContactTask, this, _1), command_options);

        if (tcp_pose_planning_enabled_)
        {
            tcp_poses_plan_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
                "tcp_poses_plan_target", 10, std::bind(&MTCTestTask::createTcpTask, this, _1), command_options);
            execute_tcp_plan_sub_ = node_->create_subscription<std_msgs::msg::String>(
                "execute_tcp_plan", 10, std::bind(&MTCTestTask::executeTcpPlan, this, _1), command_options);
        }

        execute_task_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "execute_task", 10, std::bind(&MTCTestTask::doTask, this, _1), command_options);

        // Cache the latest joint state so we can capture the arm configuration at planning
        // time and use it as the task's "return to start" goal (see createTask). Use a
        // best-effort QoS, which is compatible with both reliable and best-effort publishers.
        //
        // Keep high-rate feedback in a separate callback group and protect its
        // cache independently of the command mutex, including during service waits.
        joint_state_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions joint_state_options;
        joint_state_options.callback_group = joint_state_cb_group_;
        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::JointState::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(joint_state_mutex_);
                latest_joint_state_ = msg;
                latest_joint_state_received_ = std::chrono::steady_clock::now();
            },
            joint_state_options);

        // Outputs for the Unity / Magic Leap 2 app. transient_local so a late-joining
        // subscriber (or the ROS-TCP endpoint) still receives the most recent plan.
        planned_trajectory_pub_ = node_->create_publisher<moveit_msgs::msg::RobotTrajectory>(
            "/mtc/planned_trajectory", rclcpp::QoS(1).transient_local());
        planning_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/mtc/planning_status", rclcpp::QoS(1).transient_local());
        execution_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/mtc/execution_status", rclcpp::QoS(1).transient_local());
        tcp_planned_trajectory_pub_ = node_->create_publisher<moveit_msgs::msg::RobotTrajectory>(
            "/mtc/tcp_planned_trajectory", rclcpp::QoS(1).transient_local());
        tcp_planning_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/mtc/tcp_planning_status", rclcpp::QoS(1).transient_local());
        tcp_execution_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/mtc/tcp_execution_status", rclcpp::QoS(1).transient_local());
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
    getNodeBaseInterface()
    {
        return node_->get_node_base_interface();
    }

private:
    using GetPlanningScene = moveit_msgs::srv::GetPlanningScene;
    using ApplyPlanningScene = moveit_msgs::srv::ApplyPlanningScene;
    using TaskMode = ur16e_mtc_gpio_stage::PlannedTaskMode;
    struct PreparedPlan
    {
        TaskMode mode = TaskMode::VacuumPickPlace;
        std::string request_id;
        moveit_task_constructor_msgs::msg::Solution solution;
        moveit_msgs::msg::PlanningScene start_scene;
        std::string robot_description;
        std::string robot_description_semantic;
    };

    rclcpp::Node::SharedPtr node_;
    const ur16e_mtc_gpio_stage::ToolDefinition tool_;
    const std::string ik_frame_;
    const bool tcp_pose_planning_enabled_;
    TaskMode active_task_mode_ = TaskMode::VacuumPickPlace;
    builtin_interfaces::msg::Time active_tcp_request_stamp_;
    std::string active_tcp_request_id_;
    moveit::core::RobotModelConstPtr startup_model_;
    mtc::Task task_;
    // This mutex protects task_, gate_ and the immutable prepared plan for the
    // entire callback, including waits. Busy callbacks do not access these fields.
    std::mutex task_mutex_;
    ur16e_mtc_gpio_stage::PlanGate gate_;
    std::unique_ptr<const PreparedPlan> prepared_plan_;
    rclcpp::CallbackGroup::SharedPtr command_cb_group_;
    rclcpp::CallbackGroup::SharedPtr scene_client_cb_group_;
    rclcpp::Client<GetPlanningScene>::SharedPtr scene_query_client_;
    rclcpp::Client<ApplyPlanningScene>::SharedPtr scene_apply_client_;
    double start_joint_tolerance_;
    double joint_state_max_age_;
    std::string startup_robot_description_;
    std::string startup_robot_description_semantic_;
    rclcpp::Subscription<ur16e_interfaces::msg::PickPlacePoses>::SharedPtr pick_place_sub_;
    rclcpp::Subscription<ur16e_interfaces::msg::PickPlacePoses>::SharedPtr pick_place_tcp_sub_;
    std::optional<moveit_msgs::msg::CollisionObject> owned_preview_object_;
    rclcpp::Subscription<ur16e_interfaces::msg::Box>::SharedPtr scene_setup_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr tcp_poses_plan_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr execute_tcp_plan_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execute_task_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::CallbackGroup::SharedPtr joint_state_cb_group_;
    sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
    std::chrono::steady_clock::time_point latest_joint_state_received_;
    std::mutex joint_state_mutex_;

    // Unity-facing outputs (consumed by the Magic Leap 2 app over ROS-TCP):
    //  - planned_trajectory: the chosen MTC solution flattened to a single joint path,
    //    so the headset can preview/animate the robot hologram along the plan.
    //  - planning_status: compact JSON (overall success + per-stage solved/failed),
    //    so the headset can show the planning steps like the RViz task panel.
    rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr planned_trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planning_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr execution_status_pub_;
    rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr tcp_planned_trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tcp_planning_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tcp_execution_status_pub_;

    void publishExecutionStatus(const std::string &status, const std::string &reason)
    {
        try
        {
            std_msgs::msg::String message;
            message.data = "{\"status\":\"" + jsonEscape(status) + "\",\"reason\":\"" + jsonEscape(reason) + "\"}";
            execution_status_pub_->publish(message);
        }
        catch (const std::exception &error)
        {
            RCLCPP_ERROR(LOGGER, "Could not publish execution status: %s", error.what());
        }
        RCLCPP_INFO(LOGGER, "Execution %s: %s", status.c_str(), reason.c_str());
    }

    void publishTcpExecutionStatus(const std::string &status, const std::string &reason,
                                   const std::string &request_id)
    {
        try
        {
            std_msgs::msg::String message;
            message.data = "{\"status\":\"" + jsonEscape(status) + "\",\"reason\":\"" + jsonEscape(reason) +
                "\",\"request_id\":\"" + jsonEscape(request_id) + "\"}";
            tcp_execution_status_pub_->publish(message);
        }
        catch (const std::exception &error)
        {
            RCLCPP_ERROR(LOGGER, "Could not publish TCP execution status: %s", error.what());
        }
        RCLCPP_INFO(LOGGER, "TCP execution %s [%s]: %s", status.c_str(), request_id.c_str(), reason.c_str());
    }

    void discardPreparedPlan()
    {
        prepared_plan_.reset();
        planned_trajectory_pub_->publish(moveit_msgs::msg::RobotTrajectory{});
        moveit_msgs::msg::RobotTrajectory empty_tcp;
        empty_tcp.joint_trajectory.header.stamp = active_tcp_request_stamp_;
        tcp_planned_trajectory_pub_->publish(empty_tcp);
    }

    void selectTaskMode(TaskMode mode)
    {
        // Both modes share one frozen plan. Invalidate the other retained status
        // so another UI cannot mistake an older task's success for readiness.
        if (active_task_mode_ != mode)
            publishPlanningStatus(false, "Replaced by a different planning mode", "invalidated");
        active_task_mode_ = mode;
    }

    // Called only under the command mutex, before planning or execution admission.
    // Revoking readiness first also protects against accidentally retained storage.
    void rejectVacuumRequest(const std::string &reason)
    {
        gate_.invalidate();
        prepared_plan_.reset();
        try
        {
            task_ = mtc::Task();
            discardPreparedPlan();
            publishPlanningStatus(false);
        }
        catch (const std::exception &error)
        {
            RCLCPP_ERROR(LOGGER, "Could not clear rejected task preview/status: %s", error.what());
        }
        publishExecutionStatus("rejected", reason);
    }

    std::string toolConfigurationError(bool require_vacuum = true) const
    {
        if (require_vacuum && !tool_.vacuum)
            return "Mounted tool '" + tool_.name + "' does not support vacuum pick/place";
        if (node_->get_parameter_or<std::string>("tool", "none") != tool_.name ||
            node_->get_parameter_or<std::string>("ik_frame", "tcp") != ik_frame_)
            return "Tool selection or TCP changed since startup; restart the task node";
        return {};
    }

    template <class Service>
    typename Service::Response::SharedPtr callSceneService(
        const typename rclcpp::Client<Service>::SharedPtr &client,
        const typename Service::Request::SharedPtr &request)
    {
        using namespace std::chrono_literals;
        if (!client->wait_for_service(2s))
            throw std::runtime_error(std::string(client->get_service_name()) + " is unavailable");
        auto future = client->async_send_request(request);
        if (future.wait_for(3s) != std::future_status::ready)
        {
            client->remove_pending_request(future);
            throw std::runtime_error(std::string(client->get_service_name()) +
                                     " timed out; inspect the scene before retrying a scene update");
        }
        auto response = future.get();
        if (!response)
            throw std::runtime_error(std::string(client->get_service_name()) + " returned no response");
        return response;
    }

    std::string validatePreparedPlan(const PreparedPlan &plan)
    {
        if (node_->get_parameter_or<std::string>("robot_description", "") != plan.robot_description ||
            node_->get_parameter_or<std::string>("robot_description_semantic", "") != plan.robot_description_semantic)
            return "Robot/tool description changed; replan";

        auto request = std::make_shared<GetPlanningScene::Request>();
        using Components = moveit_msgs::msg::PlanningSceneComponents;
        request->components.components = Components::SCENE_SETTINGS | Components::ROBOT_STATE |
            Components::ROBOT_STATE_ATTACHED_OBJECTS | Components::WORLD_OBJECT_NAMES |
            Components::WORLD_OBJECT_GEOMETRY | Components::OCTOMAP | Components::TRANSFORMS |
            Components::ALLOWED_COLLISION_MATRIX | Components::LINK_PADDING_AND_SCALING;
        const auto response = callSceneService<GetPlanningScene>(scene_query_client_, request);
        const auto scene_reason = ur16e_mtc_gpio_stage::validatePlanStart(
            plan.start_scene, response->scene, start_joint_tolerance_);
        if (!scene_reason.empty())
            return scene_reason;

        sensor_msgs::msg::JointState feedback;
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);
            if (!latest_joint_state_ ||
                std::chrono::duration<double>(std::chrono::steady_clock::now() - latest_joint_state_received_).count() >
                    joint_state_max_age_)
                return "Joint-state feedback is missing or stale; replan with current feedback";
            feedback = *latest_joint_state_;
        }
        return ur16e_mtc_gpio_stage::validateJointStart(
            plan.start_scene.robot_state.joint_state, feedback, start_joint_tolerance_);
    }

    void printPickPlacePoses(ur16e_interfaces::msg::PickPlacePoses poses)
    {
        RCLCPP_DEBUG(LOGGER, "PickPlacePoses received:");
        RCLCPP_DEBUG(LOGGER, "  Pick pose frame_id: '%s'", poses.pick_pose.header.frame_id.c_str());
        RCLCPP_DEBUG(LOGGER, "  Pick pose position: x=%f, y=%f, z=%f",
                     poses.pick_pose.pose.position.x,
                     poses.pick_pose.pose.position.y,
                     poses.pick_pose.pose.position.z);
        RCLCPP_DEBUG(LOGGER, "  Pick pose orientation: x=%f, y=%f, z=%f, w=%f",
                     poses.pick_pose.pose.orientation.x,
                     poses.pick_pose.pose.orientation.y,
                     poses.pick_pose.pose.orientation.z,
                     poses.pick_pose.pose.orientation.w);
        RCLCPP_DEBUG(LOGGER, "  Place pose frame_id: '%s'", poses.place_pose.header.frame_id.c_str());
        RCLCPP_DEBUG(LOGGER, "  Place pose position: x=%f, y=%f, z=%f",
                     poses.place_pose.pose.position.x,
                     poses.place_pose.pose.position.y,
                     poses.place_pose.pose.position.z);
        RCLCPP_DEBUG(LOGGER, "  Place pose orientation: x=%f, y=%f, z=%f, w=%f",
                     poses.place_pose.pose.orientation.x,
                     poses.place_pose.pose.orientation.y,
                     poses.place_pose.pose.orientation.z,
                     poses.place_pose.pose.orientation.w);
    }


    ur16e_interfaces::msg::Box createBoxMessage(PoseStamped pose)
    {
        ur16e_interfaces::msg::Box box_msg;
        box_msg.boxx = tile_size;
        box_msg.boxy = tile_size;
        box_msg.boxz = tile_height;
        box_msg.position.header.frame_id = pose.header.frame_id;
        box_msg.position.pose.position.x = pose.pose.position.x;
        box_msg.position.pose.position.y = pose.pose.position.y;
        box_msg.position.pose.position.z = pose.pose.position.z;
        box_msg.position.pose.orientation.x = pose.pose.orientation.x;
        box_msg.position.pose.orientation.y = pose.pose.orientation.y;
        box_msg.position.pose.orientation.z = pose.pose.orientation.z;
        box_msg.position.pose.orientation.w = pose.pose.orientation.w;
        return box_msg;
    }

    void setupPlanningScene(ur16e_interfaces::msg::Box box_msg)
    {
        std::unique_lock<std::mutex> lock(task_mutex_, std::try_to_lock);
        if (!lock.owns_lock())
        {
            publishExecutionStatus("rejected", "Scene update rejected: task is busy");
            return;
        }
        gate_.invalidate();
        try
        {
            discardPreparedPlan();
            applyBox(box_msg);
            publishExecutionStatus("invalidated", "Scene updated; a new plan is required");
        }
        catch (const std::exception &error)
        {
            publishExecutionStatus("failed", std::string("Scene update failed; plan invalidated: ") + error.what());
        }
    }

    void applyBox(const ur16e_interfaces::msg::Box &box_msg, bool task_preview = false)
    {
        owned_preview_object_.reset(); // a failed/unknown scene write revokes prior ownership too
        RCLCPP_INFO(LOGGER, "Setting up planning scene with box dimensions: x=%f, y=%f, z=%f", box_msg.boxx, box_msg.boxy, box_msg.boxz);

        moveit_msgs::msg::CollisionObject object;
        object.id = "object";
        object.header.frame_id = box_msg.position.header.frame_id;
        object.primitives.resize(1);
        object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        object.primitives[0].dimensions = {box_msg.boxx, box_msg.boxy, box_msg.boxz}; 
        object.pose = box_msg.position.pose;
        object.primitive_poses.resize(1);
        object.primitive_poses[0].orientation.w = 1.0;
        object.operation = object.ADD;

        auto request = std::make_shared<ApplyPlanningScene::Request>();
        request->scene.is_diff = true;
        request->scene.robot_state.is_diff = true;
        request->scene.world.collision_objects.push_back(std::move(object));
        if (!callSceneService<ApplyPlanningScene>(scene_apply_client_, request)->success)
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

    void doTask(std_msgs::msg::Bool execute_msg)
    {
        if (!execute_msg.data)
        {
            RCLCPP_INFO(LOGGER, "Execute task message is false, not executing.");
            return;
        }
        executePreparedPlan(false, "");
    }

    void executeTcpPlan(std_msgs::msg::String request)
    {
        executePreparedPlan(true, request.data);
    }

    void executePreparedPlan(bool tcp_request, const std::string &request_id)
    {
        // Correlate rejection with the requested ID, even if a different plan is active.
        const auto report = [&](const std::string &status, const std::string &reason) {
            if (tcp_request) publishTcpExecutionStatus(status, reason, request_id);
            else publishExecutionStatus(status, reason);
        };
        std::unique_lock<std::mutex> lock(task_mutex_, std::try_to_lock);
        if (!lock.owns_lock())
        {
            report("rejected", "Task is busy");
            return;
        }
        const auto requested_mode = tcp_request ? TaskMode::TcpWaypoints : TaskMode::VacuumPickPlace;
        if (prepared_plan_ && !ur16e_mtc_gpio_stage::matchesExecutionRequest(
                prepared_plan_->mode, prepared_plan_->request_id, requested_mode, request_id))
        {
            report("rejected", "Execution mode or request ID does not match the prepared plan");
            return;
        }
        if (tcp_request && !prepared_plan_)
        {
            report("rejected", "No successful TCP plan matches the requested ID");
            return;
        }
        const auto tool_error = toolConfigurationError(!tcp_request);
        if (!tool_error.empty())
        {
            rejectVacuumRequest(tool_error);
            if (tcp_request) report("rejected", tool_error);
            return;
        }
        if (!gate_.beginExecution(static_cast<bool>(prepared_plan_)))
        {
            report("rejected", "No unconsumed successful plan; request a new plan first");
            return;
        }

        // Consume readiness BEFORE constructing clients or validating. Any failed
        // attempt requires fresh planning; the immutable message stays alive locally.
        auto plan = std::move(prepared_plan_);
        try
        {
            discardPreparedPlan();
            const bool vacuum_task = plan->mode == TaskMode::VacuumPickPlace;
            // After any vacuum execution attempt, the object may be physical or
            // moved/attached. Never automatically remove it as a preview again.
            if (vacuum_task) owned_preview_object_.reset();
            const std::string io_service = vacuum_task
                ? ur16e_mtc_gpio_stage::requireVacuumCapability(tool_).io_service
                : "/io_and_status_controller/set_io";
            // TCP plans have no IO stages; the executor also forbids IO explicitly.
            ur16e_mtc_gpio_stage::CustomExecutor exec(node_, "ur_arm", io_service, vacuum_task);
            const auto reason = validatePreparedPlan(*plan);
            if (!reason.empty())
            {
                gate_.finishExecution(false);
                report("rejected", reason);
                return;
            }
            report("executing", "Start state and scene validated; plan consumed");
            const bool success = exec.executeSolution(plan->solution);
            gate_.finishExecution(success);
            report(success ? "succeeded" : "failed",
                success ? "Task completed; a new plan is required for another execution" :
                          "Execution stopped; inspect robot and scene before planning again");
        }
        catch (const std::exception &error)
        {
            gate_.finishExecution(false);
            report("failed", std::string("Execution attempt stopped; plan consumed: ") + error.what());
        }
    }

    void planTask(const std::string &robot_description, const std::string &robot_description_semantic)
    {
        if (!task_.plan(2) || task_.solutions().empty() || !task_.solutions().front())
            throw std::runtime_error("Task planning produced no successful solution");
        const auto &best = *task_.solutions().front();
        if (!best.start() || !best.start()->scene())
            throw std::runtime_error("Successful solution is missing its start scene");
        if (node_->get_parameter_or<std::string>("robot_description", "") != robot_description ||
            node_->get_parameter_or<std::string>("robot_description_semantic", "") != robot_description_semantic)
            throw std::runtime_error("Robot/tool description changed during planning; restart with consistent inputs");
        auto prepared = std::make_unique<PreparedPlan>();
        prepared->mode = active_task_mode_;
        prepared->request_id = active_task_mode_ == TaskMode::TcpWaypoints ? active_tcp_request_id_ : "";
        best.toMsg(prepared->solution);
        if (prepared->solution.sub_trajectory.empty())
            throw std::runtime_error("Successful solution contains no executable steps");
        best.start()->scene()->getPlanningSceneMsg(prepared->start_scene);
        prepared->robot_description = robot_description;
        prepared->robot_description_semantic = robot_description_semantic;
        prepared_plan_ = std::move(prepared);
        task_.introspection().publishSolution(best);
        // Push the plan to the Unity / Magic Leap 2 app: the trajectory for the hologram
        // preview, and a per-stage status summary for the headset UI.
        publishPlannedTrajectory(prepared_plan_->solution);
        publishPlanningStatus(true, "Exact TCP A then B plan ready; no motion or IO sent");
        publishExecutionStatus("ready", "A successful plan is available for one execution attempt");
        gate_.finishPlanning(true);
    }

    // Minimal JSON string escaping for stage names embedded in the status payload.
    static std::string jsonEscape(const std::string &s)
    {
        std::string out;
        out.reserve(s.size() + 8);
        for (char c : s)
        {
            switch (c)
            {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default: out += c;
            }
        }
        return out;
    }

    // Publish a compact JSON summary of the task's stages (name / depth / #solved /
    // #failed) plus an overall success flag, on /mtc/planning_status (std_msgs/String).
    // The Unity app parses this to render the planning steps like the RViz task panel.
    void publishPlanningStatus(bool overall_success, const std::string &message = "", const std::string &phase = "")
    {
        std::ostringstream oss;
        oss << "{\"overall_success\":" << (overall_success ? "true" : "false")
            << ",\"stages\":[";
        bool first = true;
        const mtc::ContainerBase *stages = task_.stages();
        if (stages)
        {
            stages->traverseRecursively(
                [&](const mtc::Stage &stage, unsigned int depth) -> bool
                {
                    if (!first)
                        oss << ",";
                    first = false;
                    oss << "{\"name\":\"" << jsonEscape(stage.name()) << "\""
                        << ",\"depth\":" << depth
                        << ",\"solved\":" << stage.solutions().size()
                        << ",\"failed\":" << stage.numFailures() << "}";
                    return true; // keep traversing
                });
        }
        if (active_task_mode_ == TaskMode::TcpWaypoints)
            oss << "],\"message\":\"" << jsonEscape(message)
                << "\",\"request_id\":\"" << jsonEscape(active_tcp_request_id_)
                << "\",\"phase\":\"" << jsonEscape(phase.empty() ? (overall_success ? "ready" : "failed") : phase) << "\"}";
        else
            oss << "]}";

        std_msgs::msg::String msg;
        msg.data = oss.str();
        (active_task_mode_ == TaskMode::TcpWaypoints ? tcp_planning_status_pub_ : planning_status_pub_)->publish(msg);
        RCLCPP_INFO(LOGGER, "Published planning status (overall_success=%s) to %s",
                    overall_success ? "true" : "false", active_task_mode_ == TaskMode::TcpWaypoints
                        ? "/mtc/tcp_planning_status" : "/mtc/planning_status");
    }

    // Flatten the best MTC solution into a single moveit_msgs/RobotTrajectory and publish
    // it on /mtc/planned_trajectory. We concatenate the joint trajectories of all motion
    // sub-trajectories (skipping IO-marker / scene-diff subs that carry no motion),
    // re-basing time_from_start cumulatively, and reordering each segment's positions to a
    // single consistent joint_names order. This matches what the Unity
    // UR16eTrajectoryVisualizer expects (joint_trajectory.joint_names + points[].positions).
    void publishPlannedTrajectory(const moveit_task_constructor_msgs::msg::Solution &sol_msg)
    {
        moveit_msgs::msg::RobotTrajectory flat;
        bool have_names = false;
        double t_offset = 0.0;

        for (const auto &sub : sol_msg.sub_trajectory)
        {
            const auto &jt = sub.trajectory.joint_trajectory;
            if (jt.points.empty() || jt.joint_names.empty())
                continue; // IO markers / scene diffs carry no motion

            if (!have_names)
            {
                flat.joint_trajectory.header = jt.header;
                flat.joint_trajectory.joint_names = jt.joint_names;
                have_names = true;
            }

            // Map: for each output joint, its index within this segment's joint_names.
            std::vector<int> idx(flat.joint_trajectory.joint_names.size(), -1);
            for (std::size_t f = 0; f < flat.joint_trajectory.joint_names.size(); ++f)
                for (std::size_t s = 0; s < jt.joint_names.size(); ++s)
                    if (jt.joint_names[s] == flat.joint_trajectory.joint_names[f])
                    {
                        idx[f] = static_cast<int>(s);
                        break;
                    }

            double seg_last = 0.0;
            for (const auto &pt : jt.points)
            {
                trajectory_msgs::msg::JointTrajectoryPoint np;
                np.positions.resize(flat.joint_trajectory.joint_names.size());
                bool ok = true;
                for (std::size_t f = 0; f < idx.size(); ++f)
                {
                    if (idx[f] < 0 || static_cast<std::size_t>(idx[f]) >= pt.positions.size())
                    {
                        ok = false;
                        break;
                    }
                    np.positions[f] = pt.positions[idx[f]];
                }
                if (!ok)
                    continue;

                const double tfs = static_cast<double>(pt.time_from_start.sec) +
                                   static_cast<double>(pt.time_from_start.nanosec) * 1e-9;
                const double newt = t_offset + tfs;
                np.time_from_start.sec = static_cast<int32_t>(newt);
                np.time_from_start.nanosec =
                    static_cast<uint32_t>((newt - std::floor(newt)) * 1e9);
                flat.joint_trajectory.points.push_back(std::move(np));
                seg_last = tfs;
            }
            t_offset += seg_last;
        }

        if (flat.joint_trajectory.points.empty())
        {
            RCLCPP_WARN(LOGGER, "Flattened planned trajectory has no points; nothing published");
            return;
        }

        if (active_task_mode_ == TaskMode::TcpWaypoints)
            flat.joint_trajectory.header.stamp = active_tcp_request_stamp_;
        (active_task_mode_ == TaskMode::TcpWaypoints ? tcp_planned_trajectory_pub_ : planned_trajectory_pub_)->publish(flat);
        RCLCPP_INFO(LOGGER,
                    "Published planned trajectory (%zu points, %zu joints) to %s",
                    flat.joint_trajectory.points.size(),
                    flat.joint_trajectory.joint_names.size(), active_task_mode_ == TaskMode::TcpWaypoints
                        ? "/mtc/tcp_planned_trajectory" : "/mtc/planned_trajectory");
    }

    void createTask(ur16e_interfaces::msg::PickPlacePoses poses)
    {
        createTaskImpl(std::move(poses), false);
    }

    void createContactTask(ur16e_interfaces::msg::PickPlacePoses poses)
    {
        createTaskImpl(std::move(poses), true);
    }

    void createTaskImpl(ur16e_interfaces::msg::PickPlacePoses poses, bool tool_contacts)
    {
        std::unique_lock<std::mutex> lock(task_mutex_, std::try_to_lock);
        if (!lock.owns_lock())
        {
            publishExecutionStatus("rejected", "Planning request rejected: task is busy");
            return;
        }
        selectTaskMode(TaskMode::VacuumPickPlace);
        const auto tool_error = toolConfigurationError();
        if (!tool_error.empty())
        {
            rejectVacuumRequest(tool_error);
            return;
        }
        if (!gate_.beginPlanning())
        {
            publishExecutionStatus("rejected", "Planning request rejected: task is busy");
            return;
        }
        try
        {
            discardPreparedPlan();
            task_ = mtc::Task();
            task_.setRobotModel(startup_model_);
            publishPlanningStatus(false);
            publishExecutionStatus("invalidated", "New planning request replaced the previous plan");
            const auto robot_description = node_->get_parameter_or<std::string>("robot_description", "");
            const auto robot_description_semantic =
                node_->get_parameter_or<std::string>("robot_description_semantic", "");
            if (robot_description.empty() || robot_description_semantic.empty())
                throw std::runtime_error("Set robot_description and robot_description_semantic before planning");
            if (robot_description != startup_robot_description_ ||
                robot_description_semantic != startup_robot_description_semantic_)
                throw std::runtime_error("Robot/tool descriptions changed since startup; restart the task node");
            if (tool_contacts)
                poses = ur16e_mtc_gpio_stage::validatedContactTargets(std::move(poses));
            buildTask(poses, tool_contacts);
            task_.init();
            planTask(robot_description, robot_description_semantic);
        }
        catch (const mtc::InitStageException &error)
        {
            std::ostringstream message;
            message << error;
            planningFailed(message.str());
        }
        catch (const std::exception &error)
        {
            planningFailed(error.what());
        }
    }

    // A separate plan-only endpoint. These are exact TCP A/B goals, not tiles.
    // This callback never executes a solution and never adds vacuum/object stages.
    void createTcpTask(geometry_msgs::msg::PoseArray request)
    {
        std::unique_lock<std::mutex> lock(task_mutex_, std::try_to_lock);
        if (!lock.owns_lock())
        {
            std_msgs::msg::String failure;
            failure.data = "{\"overall_success\":false,\"phase\":\"failed\",\"stages\":[],\"message\":\"Task is busy\",\"request_id\":\"" +
                ur16e_mtc_gpio_stage::tcpRequestId(request.header.stamp) + "\"}";
            tcp_planning_status_pub_->publish(failure);
            publishExecutionStatus("rejected", "TCP planning request rejected: task is busy");
            return;
        }
        selectTaskMode(TaskMode::TcpWaypoints);
        active_tcp_request_stamp_ = request.header.stamp;
        active_tcp_request_id_ = ur16e_mtc_gpio_stage::tcpRequestId(request.header.stamp);
        const auto tool_error = toolConfigurationError(false);
        if (!tool_error.empty())
        {
            rejectVacuumRequest(tool_error);
            return;
        }
        if (!gate_.beginPlanning())
        {
            publishExecutionStatus("rejected", "TCP planning request rejected: task is busy");
            return;
        }
        try
        {
            discardPreparedPlan();
            task_ = mtc::Task();
            task_.setRobotModel(startup_model_);
            publishPlanningStatus(false, "Planning exact TCP A then B; no motion or IO sent", "planning");
            publishExecutionStatus("invalidated", "TCP planning request replaced the previous plan");
            request = ur16e_mtc_gpio_stage::validatedTcpWaypoints(std::move(request));
            const auto description = node_->get_parameter_or<std::string>("robot_description", "");
            const auto semantic = node_->get_parameter_or<std::string>("robot_description_semantic", "");
            if (description != startup_robot_description_ || semantic != startup_robot_description_semantic_)
                throw std::runtime_error("Robot/tool descriptions changed since startup; restart the task node");
            removeUnexecutedPreviewObject();
            task_.stages()->setName("Exact TCP A then B");
            task_.setProperty("group", "ur_arm");
            task_.setProperty("ik_frame", ik_frame_);
            task_.add(std::make_unique<mtc::stages::CurrentState>("current"));
            auto planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
            planner->setTimeout(5.0);
            planner->setMaxVelocityScalingFactor(0.5);
            planner->setMaxAccelerationScalingFactor(0.5);
            for (std::size_t index = 0; index < request.poses.size(); ++index)
            {
                auto stage = std::make_unique<mtc::stages::MoveTo>(index == 0 ? "TCP A" : "TCP B", planner);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                stage->setIKFrame(ik_frame_);
                stage->setTimeout(10.0);
                PoseStamped goal;
                goal.header = request.header;
                // Request stamp is correlation only, never a historical TF time.
                goal.header.stamp = builtin_interfaces::msg::Time{};
                goal.pose = request.poses[index];
                stage->setGoal(goal);
                task_.add(std::move(stage));
            }
            task_.init();
            planTask(description, semantic);
        }
        catch (const mtc::InitStageException &error)
        {
            std::ostringstream message;
            message << error;
            planningFailed(message.str());
        }
        catch (const std::exception &error)
        {
            planningFailed(error.what());
        }
    }

    void planningFailed(const std::string &reason)
    {
        gate_.finishPlanning(false);
        prepared_plan_.reset();
        RCLCPP_ERROR(LOGGER, "Task construction, initialization or planning failed: %s", reason.c_str());
        // A reporting error must not restore readiness or let the original
        // planning exception escape the subscription callback.
        try
        {
            discardPreparedPlan();
            publishPlanningStatus(false, reason);
            publishExecutionStatus("planning_failed", reason);
        }
        catch (const std::exception &error)
        {
            RCLCPP_ERROR(LOGGER, "Could not publish planning failure: %s", error.what());
        }
    }

    void buildTask(const ur16e_interfaces::msg::PickPlacePoses &poses, bool tool_contacts)
    {
        // This prerequisite is checked before even the first scene update. The
        // startup model and tool definition remain fixed throughout the session.
        const auto &vacuum = ur16e_mtc_gpio_stage::requireVacuumCapability(tool_);
        printPickPlacePoses(poses);

        PoseStamped tile_pose = poses.pick_pose;
        if (tool_contacts)
            tile_pose = ur16e_mtc_gpio_stage::contactObjectCentre(tile_pose, tile_height);
        else
            tile_pose.pose.position.z -= tile_height / 2; // legacy horizontal surface convention
        ur16e_interfaces::msg::Box b = createBoxMessage(tile_pose);
        applyBox(b, true);

        task_.stages()->setName(tool_contacts ? "Tool contact pick/place" : "UR GPIO + Motion");

        const auto &arm_group_name = "ur_arm";

        task_.setProperty("group", arm_group_name);
        task_.setProperty("ik_frame", ik_frame_);

        mtc::Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
        current_state_ptr = stage_state_current.get();
        task_.add(std::move(stage_state_current));
        RCLCPP_DEBUG(LOGGER, "CurrentState stage added");

        // Planners creation and configuration
        auto sampling_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
        sampling_planner_->setTimeout(5.0);  // cap each OMPL attempt at 5 s; failing connects no longer burn 30 s
        auto interpolation_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
        interpolation_planner_->setTimeout(15.0);  // allow more time for "return home" stage
        interpolation_planner_->setMaxVelocityScalingFactor(0.5);
        interpolation_planner_->setMaxAccelerationScalingFactor(0.5);
        // Relaxed Cartesian planner (min_fraction 0.9): used for non-contact positioning
        // moves where stopping a little short is harmless (e.g. the initial pre-IK
        // approach). Note: MoveRelative stages with a min/max *range* (lift, retreat)
        // ignore min_fraction entirely - their success is gated by min_distance - so this
        // relaxation only affects fixed-distance moves.
        auto cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner_->setMaxVelocityScalingFactor(0.5);
        cartesian_planner_->setMaxAccelerationScalingFactor(0.5);
        cartesian_planner_->setStepSize(0.01);
        cartesian_planner_->setMinFraction(0.9);

        // Strict Cartesian planner (min_fraction 1.0): used for contact-critical moves
        // (seating the suction cup onto the tile, and lowering the tile to the place
        // surface). A partial path here would leave the cup ~1 cm short -> no vacuum seal
        // / tile placed too high, so we require the full motion.
        auto cartesian_planner_strict_ = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner_strict_->setMaxVelocityScalingFactor(0.5);
        cartesian_planner_strict_->setMaxAccelerationScalingFactor(0.5);
        cartesian_planner_strict_->setStepSize(0.01);
        cartesian_planner_strict_->setMinFraction(1.0);

        // Connector stage: will try to bridge the previous state (current state)
        // to the next state (which we'll define as the 'pick object' container below)
        // The pick object container first stage will be a MoveRelative stage, i.e. a propagator stage.
        // In this case, it will propagate backwards from the stage after it to the connect stage here.
        auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
            "move to pick",
            mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner_}});
        stage_move_to_pick->setTimeout(10.0);  // give Connect stage up to 10 s to find a path
        stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
        task_.add(std::move(stage_move_to_pick));
        RCLCPP_DEBUG(LOGGER, "Connect stage 'move to pick' added");

        mtc::Stage *attach_object_stage = nullptr; // Forward attach_object_stage to place pose generator
        {
            // Container for pick operation
            auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
            RCLCPP_DEBUG(LOGGER, "Configuring pick object container properties");
            task_.properties().exposeTo(grasp->properties(), {"group", "ik_frame"});
            grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});

            {
                RCLCPP_DEBUG(LOGGER, "Creating MoveRelative stage 'approach object'");
                auto stage =
                    std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner_);
                stage->properties().set("marker_ns", "approach_object");
                stage->properties().set("link", ik_frame_);
                stage->setIKFrame(ik_frame_);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                stage->setMinMaxDistance(0.1, 0.1);

                // Set hand forward direction
                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = ik_frame_;
                vec.vector.z = 1.0;
                stage->setDirection(vec);
                grasp->insert(std::move(stage));
                RCLCPP_DEBUG(LOGGER, "MoveRelative stage 'approach object' added to grasp container");
            }

            {
                RCLCPP_DEBUG(LOGGER, "Creating yaw-sampled grasp pose Alternatives");
                if (current_state_ptr == nullptr)
                {
                    RCLCPP_ERROR(LOGGER, "current_state_ptr is nullptr! This may cause issues!");
                }

                // The vacuum grasp has a free rotational DOF about the tile normal (the
                // 'object' frame +z). Rather than commit to a single wrist orientation
                // (which often lands near a singularity / joint limit and fails IK), we
                // sample that yaw and let ComputeIK pick whichever orientations are
                // reachable. Each branch is the exact GeneratePose+ComputeIK pattern,
                // merged by an Alternatives container.
                auto alternatives = std::make_unique<mtc::Alternatives>("grasp pose alternatives");
                task_.properties().exposeTo(alternatives->properties(), {"group", "ik_frame"});
                alternatives->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});

                const int n_yaw_samples = 12;  // 30 deg steps over a full turn
                for (int i = 0; i < n_yaw_samples; ++i)
                {
                    const double yaw = 2.0 * M_PI * static_cast<double>(i) / n_yaw_samples;

                    auto gen = std::make_unique<mtc::stages::GeneratePose>("grasp pose " + std::to_string(i));
                    gen->properties().configureInitFrom(mtc::Stage::PARENT);
                    gen->properties().set("marker_ns", "grasp_pose");

                    geometry_msgs::msg::PoseStamped target_pose_msg;
                    target_pose_msg.header.frame_id = "object";
                    target_pose_msg.pose.position.x = 0.0;
                    target_pose_msg.pose.position.y = 0.0;
                    target_pose_msg.pose.position.z = -approach_distance - tile_height / 2; // only valid if reference frame has same +z as tile
                    // Rotation about the object's +z axis (the tile normal): only the wrist
                    // yaw changes, position stays on the approach axis.
                    target_pose_msg.pose.orientation.x = 0.0;
                    target_pose_msg.pose.orientation.y = 0.0;
                    target_pose_msg.pose.orientation.z = std::sin(yaw / 2.0);
                    target_pose_msg.pose.orientation.w = std::cos(yaw / 2.0);
                    gen->setPose(target_pose_msg);
                    gen->setMonitoredStage(current_state_ptr);

                    auto wrapper =
                        std::make_unique<mtc::stages::ComputeIK>("grasp pose IK " + std::to_string(i), std::move(gen));
                    // Diversity now comes from the yaw samples, so only a couple of IK
                    // solutions per pose are needed; this keeps planning time bounded.
                    wrapper->setMaxIKSolutions(2);
                    wrapper->setMinSolutionDistance(0.2);
                    wrapper->setIKFrame(ik_frame_);
                    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
                    alternatives->insert(std::move(wrapper));
                }

                grasp->insert(std::move(alternatives));
                RCLCPP_DEBUG(LOGGER, "Yaw-sampled grasp pose Alternatives added to grasp container");

                {
                    RCLCPP_DEBUG(LOGGER, "Creating ModifyPlanningScene stage 'allow collision (hand,object)'");
                    // In my case I am not closing a gripper, I am getting closer with the vacuum gripper.
                    // So I need to allow collisions between the vacuum gripper and the object BEFORE doing the cartesian approach.
                    auto stage =
                        std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
                    stage->allowCollisions("object", vacuum.contact_links, true);
                    grasp->insert(std::move(stage));
                    RCLCPP_DEBUG(LOGGER, "ModifyPlanningScene stage added to grasp container");
                }

                {
                    RCLCPP_DEBUG(LOGGER, "Creating SetIOStage 'DO ON'");
                    auto stage = std::make_unique<mtc_ur_gpio_stage::SetIOStage>(
                        node_, vacuum.io_service, "DO ON");
                    stage->properties().set<int8_t>("fun", vacuum.function);
                    stage->properties().set<int32_t>("pin", vacuum.pin);
                    stage->properties().set<double>("state", vacuum.on_state);
                    grasp->insert(std::move(stage));
                    RCLCPP_DEBUG(LOGGER, "SetIOStage 'DO ON' added to grasp container");
                }

                {
                    RCLCPP_DEBUG(LOGGER, "Creating MoveRelative stage 'finalize approach object'");
                    auto stage =
                        std::make_unique<mtc::stages::MoveRelative>("finalize approach object", cartesian_planner_strict_);
                    stage->properties().set("marker_ns", "approach_object");
                    stage->properties().set("link", ik_frame_);
                    stage->setIKFrame(ik_frame_);
                    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                    stage->setMinMaxDistance(approach_distance, approach_distance);

                    // Set hand forward direction
                    geometry_msgs::msg::Vector3Stamped vec;
                    vec.header.frame_id = ik_frame_;
                    vec.vector.z = 1.0;
                    stage->setDirection(vec);
                    grasp->insert(std::move(stage));
                    RCLCPP_DEBUG(LOGGER, "MoveRelative stage 'finalize approach object' added to grasp container");
                }

                // Attach object stage: forward pointer to be used in place container
                {
                    RCLCPP_DEBUG(LOGGER, "Creating ModifyPlanningScene stage 'attach object'");
                    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
                    stage->attachObject("object", ik_frame_);
                    attach_object_stage = stage.get();
                    grasp->insert(std::move(stage));
                    RCLCPP_DEBUG(LOGGER, "ModifyPlanningScene stage 'attach object' added to grasp container");
                }

                {
                    RCLCPP_DEBUG(LOGGER, "Creating MoveRelative stage 'lift object'");
                    auto stage =
                        std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner_);
                    stage->properties().set("marker_ns", "lift_object");
                    stage->properties().set("link", ik_frame_);
                    stage->setIKFrame(ik_frame_);
                    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                    stage->setMinMaxDistance(0.05, 0.2);

                    // Set hand forward direction
                    geometry_msgs::msg::Vector3Stamped vec;
                    vec.header.frame_id = ik_frame_;
                    vec.vector.z = -1.0;
                    stage->setDirection(vec);
                    grasp->insert(std::move(stage));
                    RCLCPP_DEBUG(LOGGER, "MoveRelative stage 'lift object' added to grasp container");
                }
            }

            RCLCPP_DEBUG(LOGGER, "Adding pick object container to task");
            task_.add(std::move(grasp));

            {
                RCLCPP_DEBUG(LOGGER, "Creating Connect stage 'move to place'");
                auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
                    "move to place",
                    mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner_}});
                stage_move_to_place->setTimeout(10.0);  // give Connect stage up to 10 s to find a path
                stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
                task_.add(std::move(stage_move_to_place));
                RCLCPP_DEBUG(LOGGER, "Connect stage 'move to place' added");
            }

            {
                RCLCPP_DEBUG(LOGGER, "Creating place object container");
                // Container for pick operation
                auto place = std::make_unique<mtc::SerialContainer>("place object");
                RCLCPP_DEBUG(LOGGER, "Configuring place object container properties");
                task_.properties().exposeTo(place->properties(), {"group", "ik_frame"});
                place->properties().configureInitFrom(mtc::Stage::PARENT, {"group", "ik_frame"});

                {
                    RCLCPP_DEBUG(LOGGER, "Creating MoveRelative stage 'approach place'");
                    auto stage =
                        std::make_unique<mtc::stages::MoveRelative>("approach place", cartesian_planner_strict_);
                    stage->properties().set("marker_ns", "approach_place");
                    stage->properties().set("link", ik_frame_);
                    stage->setIKFrame(ik_frame_);
                    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                    stage->setMinMaxDistance(approach_distance, approach_distance);

                    // Set hand forward direction
                    geometry_msgs::msg::Vector3Stamped vec;
                    vec.header.frame_id = ik_frame_;
                    vec.vector.z = 1.0;
                    stage->setDirection(vec);
                    place->insert(std::move(stage));
                    RCLCPP_DEBUG(LOGGER, "MoveRelative stage 'approach place' added to place container");
                }

                {
                    if (tool_contacts)
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
                    RCLCPP_DEBUG(LOGGER, "Creating GeneratePlacePose stage 'generate place pose'");
                    // Sample place pose
                    auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
                    stage->properties().configureInitFrom(mtc::Stage::PARENT);
                    stage->properties().set("marker_ns", "place_pose");
                    stage->setObject("object");
                    RCLCPP_DEBUG(LOGGER, "GeneratePlacePose configured with object='object'");

                    RCLCPP_DEBUG(LOGGER, "Setting place pose from PickPlacePoses message");
                    geometry_msgs::msg::PoseStamped target_pose_msg;
                    target_pose_msg.header.frame_id = poses.place_pose.header.frame_id;

                    target_pose_msg.pose.position.x = poses.place_pose.pose.position.x;
                    target_pose_msg.pose.position.y = poses.place_pose.pose.position.y;
                    // Match the pick convention: place_pose.z is the center of the tile's
                    // TOP surface. GeneratePlacePose places the object *center* at this
                    // pose, so subtract half the thickness to get the center from the top
                    // surface (only valid when the frame's +z is aligned with the tile normal).
                    target_pose_msg.pose.position.z = poses.place_pose.pose.position.z - tile_height / 2; 
                    target_pose_msg.pose.orientation.x = poses.place_pose.pose.orientation.x;
                    target_pose_msg.pose.orientation.y = poses.place_pose.pose.orientation.y;
                    target_pose_msg.pose.orientation.z = poses.place_pose.pose.orientation.z;
                    target_pose_msg.pose.orientation.w = poses.place_pose.pose.orientation.w;

                    stage->setPose(target_pose_msg);
                    RCLCPP_DEBUG(LOGGER, "Setting monitored stage to attach_object_stage");
                    if (attach_object_stage == nullptr)
                    {
                        RCLCPP_ERROR(LOGGER, "attach_object_stage is nullptr! This will cause a crash!");
                    }
                    stage->setMonitoredStage(attach_object_stage); // Hook into attach_object_stage

                    RCLCPP_DEBUG(LOGGER, "Creating ComputeIK wrapper for place pose");
                    // Compute IK
                    auto wrapper =
                        std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
                    wrapper->setMaxIKSolutions(10);
                    // Lowered from 1.0 so near-but-valid IK configurations aren't rejected
                    // as "too similar"; GeneratePlacePose already supplies the 4x 90-deg
                    // orientation candidates (square tile), so we want those solutions through.
                    wrapper->setMinSolutionDistance(0.2);
                    wrapper->setIKFrame(ik_frame_);
                    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
                    place->insert(std::move(wrapper));
                    RCLCPP_DEBUG(LOGGER, "ComputeIK wrapper added to place container");
                    }
                }

                {
                    RCLCPP_DEBUG(LOGGER, "Creating SetIOStage 'DO OFF'");
                    auto stage = std::make_unique<mtc_ur_gpio_stage::SetIOStage>(
                        node_, vacuum.io_service, "DO OFF");
                    stage->properties().set<int8_t>("fun", vacuum.function);
                    stage->properties().set<int32_t>("pin", vacuum.pin);
                    stage->properties().set<double>("state", vacuum.off_state);
                    // Give the vacuum its configured physical release time before retreat.
                    stage->properties().set<double>("post_delay", vacuum.release_delay);
                    place->insert(std::move(stage));
                    RCLCPP_DEBUG(LOGGER, "SetIOStage 'DO OFF' added to place container");
                }

                {
                    RCLCPP_DEBUG(LOGGER, "Creating ModifyPlanningScene stage 'detach object'");
                    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
                    stage->detachObject("object", ik_frame_);
                    place->insert(std::move(stage));
                    RCLCPP_DEBUG(LOGGER, "ModifyPlanningScene stage 'detach object' added to place container");
                }

                {
                    RCLCPP_DEBUG(LOGGER, "Creating MoveRelative stage 'retreat'");
                    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner_);
                    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                    stage->setMinMaxDistance(0.1, 0.3);
                    stage->properties().set("link", ik_frame_);
                    stage->setIKFrame(ik_frame_);
                    stage->properties().set("marker_ns", "retreat");

                    // Set retreat direction
                    geometry_msgs::msg::Vector3Stamped vec;
                    vec.header.frame_id = ik_frame_;
                    vec.vector.z = -1.0;
                    stage->setDirection(vec);
                    place->insert(std::move(stage));
                    RCLCPP_DEBUG(LOGGER, "MoveRelative stage 'retreat' added to place container");
                }

                {
                    RCLCPP_DEBUG(LOGGER, "Creating ModifyPlanningScene stage 'forbid collision (hand,object)'");
                    auto stage =
                        std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
                    stage->allowCollisions("object", vacuum.contact_links, false);
                    place->insert(std::move(stage));
                    RCLCPP_DEBUG(LOGGER, "ModifyPlanningScene stage 'forbid collision (hand,object)' added to place container");
                }

                RCLCPP_DEBUG(LOGGER, "Adding place object container to task");
                task_.add(std::move(place));
                RCLCPP_DEBUG(LOGGER, "Place object container added successfully");
            }

            {
                RCLCPP_DEBUG(LOGGER, "Creating MoveTo stage 'return to start'");
                // Use the sampling (OMPL) planner instead of joint interpolation: a
                // straight joint-space line back frequently sweeps the arm through the
                // table/itself and fails. OMPL searches for a collision-free path.
                auto stage = std::make_unique<mtc::stages::MoveTo>("return to start", sampling_planner_);
                stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});

                // Return to the arm configuration captured at planning time, so the robot
                // ends where the planned trajectory begins. The object/scene still change;
                // execution consumes the plan, so another request needs fresh planning.
                static const std::vector<std::string> arm_joint_names = {
                    "ur_shoulder_pan_joint", "ur_shoulder_lift_joint", "ur_elbow_joint",
                    "ur_wrist_1_joint", "ur_wrist_2_joint", "ur_wrist_3_joint"};

                std::map<std::string, double> start_joints;
                {
                    sensor_msgs::msg::JointState::SharedPtr js;
                    {
                        std::lock_guard<std::mutex> lock(joint_state_mutex_);
                        js = latest_joint_state_;
                    }
                    if (js)
                    {
                        for (std::size_t i = 0; i < js->name.size() && i < js->position.size(); ++i)
                        {
                            if (std::find(arm_joint_names.begin(), arm_joint_names.end(), js->name[i]) !=
                                arm_joint_names.end())
                                start_joints[js->name[i]] = js->position[i];
                        }
                    }
                }

                if (start_joints.size() == arm_joint_names.size())
                {
                    stage->setGoal(start_joints);
                    RCLCPP_INFO(LOGGER, "Return-to-start goal captured from current joint state");
                }
                else
                {
                    RCLCPP_WARN(LOGGER,
                                "Could not capture all arm joints from /joint_states (got %zu/6); "
                                "falling back to named 'home' goal.",
                                start_joints.size());
                    stage->setGoal("home");
                }
                task_.add(std::move(stage));
                RCLCPP_DEBUG(LOGGER, "MoveTo stage 'return to start' added");
            }
        }

    }

    PoseStamped makePoseStamped(double x, double y, double z, double roll, double pitch, double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();

        PoseStamped ps;
        ps.header.frame_id = "ur_base_link";
        ps.header.stamp = node_->get_clock()->now(); // time stamp
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = z;

        // identity orientation (w = 1)
        ps.pose.orientation.x = q.getX();
        ps.pose.orientation.y = q.getY();
        ps.pose.orientation.z = q.getZ();
        ps.pose.orientation.w = q.getW();

        return ps;
    }

    PoseStamped offsetLocalZ(const PoseStamped &in, double dz_m)
    {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        tf2::fromMsg(in.pose, T);

        Eigen::Isometry3d T_offset = Eigen::Isometry3d::Identity();
        T_offset.pretranslate(Eigen::Vector3d(0.0, 0.0, dz_m));

        Eigen::Isometry3d Tout = T * T_offset; // post-multiply = déplacement dans le repère outil

        PoseStamped out = in;
        out.pose = tf2::toMsg(Tout);
        return out;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    int result = 0;
    try
    {
        rclcpp::NodeOptions options;
        options.automatically_declare_parameters_from_overrides(true);
        auto node = std::make_shared<rclcpp::Node>("mtc_test_task_node", options);
        const auto tool = ur16e_mtc_gpio_stage::readOnlyStartupString(node, "tool", "none");
        const auto ik_frame = ur16e_mtc_gpio_stage::readOnlyStartupString(node, "ik_frame", "tcp");
        const bool tcp_planning = ur16e_mtc_gpio_stage::readOnlyStartupBool(node, "tcp_pose_planning_enabled", true);
        const bool contact_planning = ur16e_mtc_gpio_stage::readOnlyStartupBool(
            node, "pick_place_tcp_planning_enabled", true);
        MTCTestTask testTask(node, tool, ik_frame, tcp_planning, contact_planning);

        // Service responses and joint feedback remain responsive during commands.
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    }
    catch (const std::exception &error)
    {
        RCLCPP_FATAL(LOGGER, "Task node stopped: %s", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
