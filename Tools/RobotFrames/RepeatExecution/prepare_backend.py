#!/usr/bin/env python3
"""Prepare a reviewable ROS repeat patch and an isolated offline build tree."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parents[2]
ROS = Path("C:/Users/aca/source/UR16UnityControl")
PACKAGE = Path("ros_ws/src/ur16e_mtc_gpio_stage")


def replace(text, old, new):
    if text.count(old) != 1:
        raise ValueError("Expected exactly one source anchor: " + old[:90])
    return text.replace(old, new, 1)


def prepare():
    sources = {}
    for name in ("src/test_task.cpp", "include/ur16e_mtc_gpio_stage/plan_gate.hpp", "CMakeLists.txt"):
        source = ROS / PACKAGE / name
        sources[name] = source.read_text(encoding="utf-8-sig")
    task = sources["src/test_task.cpp"]
    task = replace(task, '#include "ur16e_mtc_gpio_stage/plan_gate.hpp"', '#include "ur16e_mtc_gpio_stage/plan_gate.hpp"\n#include "ur16e_mtc_gpio_stage/repeat_execution.hpp"')
    task = replace(task, '#include <optional>', '#include <optional>\n#include <atomic>\n#include <cstdint>\n#include <random>')
    task = replace(task, '        // Cache the latest joint state so we can capture the arm configuration at planning', '''        repeat_plan_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "repeat_reviewed_plan", 10, std::bind(&MTCTestTask::repeatReviewedPlan, this, _1), command_options);

        // Cache the latest joint state so we can capture the arm configuration at planning''')
    task = replace(task, '        std::string request_id;\n        moveit_task_constructor_msgs::msg::Solution solution;', '        std::string request_id;\n        std::string plan_id;\n        moveit_task_constructor_msgs::msg::Solution solution;')
    task = replace(task, '    std::unique_ptr<const PreparedPlan> prepared_plan_;', '''    std::unique_ptr<const PreparedPlan> prepared_plan_;
    // Only a confirmed successful execution can populate this immutable repeat source.
    std::unique_ptr<const PreparedPlan> completed_plan_;
    ur16e_mtc_gpio_stage::RepeatExecutionGate repeat_gate_;''')
    task = replace(task, '    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execute_task_sub_;', '    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execute_task_sub_;\n    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr repeat_plan_sub_;')
    task = replace(task, '    void publishExecutionStatus(const std::string &status, const std::string &reason)', '''    static std::string newPlanId()
    {
        static const std::string session = [] {
            std::random_device random;
            return std::string("plan-") + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) +
                "-" + std::to_string(random()) + "-" + std::to_string(random());
        }();
        static std::atomic<std::uint64_t> sequence{0};
        return session + "-" + std::to_string(++sequence);
    }

    void publishExecutionStatus(const std::string &status, const std::string &reason,
                                const std::string &plan_id = "", const std::string &attempt_id = "", bool repeatable = false)''')
    task = replace(task, '            message.data = "{\\"status\\":\\"" + jsonEscape(status) + "\\",\\"reason\\":\\"" + jsonEscape(reason) + "\\"}";', '''            message.data = "{\\"status\\":\\"" + jsonEscape(status) + "\\",\\"reason\\":\\"" + jsonEscape(reason) +
                "\\",\\"plan_id\\":\\"" + jsonEscape(plan_id) + "\\",\\"attempt_id\\":\\"" + jsonEscape(attempt_id) +
                "\\",\\"repeatable\\":" + (repeatable ? "true" : "false") + "}";''')
    task = replace(task, '                                   const std::string &request_id)\n    {', '''                                   const std::string &request_id, const std::string &plan_id = "",
                                   const std::string &attempt_id = "", bool repeatable = false)
    {''')
    task = replace(task, '                "\\",\\"request_id\\":\\"" + jsonEscape(request_id) + "\\"}";', '''                "\\",\\"request_id\\":\\"" + jsonEscape(request_id) + "\\",\\"plan_id\\":\\"" + jsonEscape(plan_id) +
                "\\",\\"attempt_id\\":\\"" + jsonEscape(attempt_id) + "\\",\\"repeatable\\":" + (repeatable ? "true" : "false") + "}";''')
    task = replace(task, '''    void discardPreparedPlan()
    {
        prepared_plan_.reset();
        planned_trajectory_pub_->publish''', '''    void discardPreparedPlan()
    {
        prepared_plan_.reset();
        completed_plan_.reset();
        repeat_gate_.invalidate();
        clearPlanPreview();
    }

    void clearPlanPreview()
    {
        planned_trajectory_pub_->publish''')
    begin = task.index('        // Consume readiness BEFORE constructing clients or validating.')
    end = task.index('\n    void planTask(', begin)
    task = task[:begin] + '''        // Legacy commands remain consumed after their first execution attempt.
        auto plan = std::move(prepared_plan_);
        completed_plan_.reset();
        repeat_gate_.invalidate();
        runPreparedPlan(std::move(plan), tcp_request, request_id, "", false);
    }

    void reportUnknownRepeat(const ur16e_mtc_gpio_stage::RepeatExecutionRequest &request, const std::string &reason)
    {
        // A missing/obsolete token has no task mode. Both topics carry the exact
        // identifiers so only the requesting client can consume its rejection.
        publishExecutionStatus("rejected", reason, request.plan_id, request.attempt_id);
        publishTcpExecutionStatus("rejected", reason, "", request.plan_id, request.attempt_id);
    }

    void repeatReviewedPlan(std_msgs::msg::String message)
    {
        ur16e_mtc_gpio_stage::RepeatExecutionRequest request;
        if (!ur16e_mtc_gpio_stage::parseRepeatExecutionRequest(message.data, request))
        {
            RCLCPP_WARN(LOGGER, "Malformed repeat execution request ignored");
            return;
        }
        std::unique_lock<std::mutex> lock(task_mutex_, std::try_to_lock);
        if (!lock.owns_lock())
        {
            // A duplicate can arrive while its execution owns the task mutex.
            // Publishing a terminal rejection for that ID would race its real
            // outcome. No busy repeat callback changes state or sends hardware IO.
            RCLCPP_WARN(LOGGER, "Repeat request ignored while task is busy");
            return;
        }
        if (repeat_gate_.isDuplicate(request)) return;
        if (!completed_plan_)
        {
            reportUnknownRepeat(request, "No retained successful plan; request a new plan");
            return;
        }
        const auto admission_error = repeat_gate_.begin(request);
        if (!admission_error.empty())
        {
            reportUnknownRepeat(request, admission_error);
            return;
        }
        if (!gate_.beginRepeatExecution(static_cast<bool>(completed_plan_)))
        {
            completed_plan_.reset();
            repeat_gate_.finish(false);
            reportUnknownRepeat(request, "Repeat admission was invalidated; request a new plan");
            return;
        }
        auto plan = std::move(completed_plan_);
        const bool tcp_request = plan->mode == TaskMode::TcpWaypoints;
        const std::string request_id = plan->request_id;
        runPreparedPlan(std::move(plan), tcp_request, request_id, request.attempt_id, true);
    }

    // Caller owns task_mutex_ for this whole operation. Every explicit repeat
    // repeats the complete preflight against the ORIGINAL frozen start scene.
    void runPreparedPlan(std::unique_ptr<const PreparedPlan> plan, bool tcp_request,
                         const std::string &request_id, const std::string &attempt_id, bool repeated)
    {
        const std::string plan_id = plan->plan_id;
        const auto report = [&](const std::string &status, const std::string &reason, bool repeatable = false) {
            if (tcp_request) publishTcpExecutionStatus(status, reason, request_id, plan_id, attempt_id, repeatable);
            else publishExecutionStatus(status, reason, plan_id, attempt_id, repeatable);
        };
        try
        {
            clearPlanPreview();
            const bool vacuum_task = plan->mode == TaskMode::VacuumPickPlace;
            const auto tool_error = toolConfigurationError(vacuum_task);
            if (!tool_error.empty()) throw std::runtime_error(tool_error);
            if (vacuum_task) owned_preview_object_.reset();
            const std::string io_service = vacuum_task
                ? ur16e_mtc_gpio_stage::requireVacuumCapability(tool_).io_service
                : "/io_and_status_controller/set_io";
            ur16e_mtc_gpio_stage::CustomExecutor exec(node_, "ur_arm", io_service, vacuum_task);
            const auto reason = validatePreparedPlan(*plan);
            if (!reason.empty())
            {
                gate_.finishExecution(false);
                repeat_gate_.finish(false);
                report("rejected", reason);
                return;
            }
            report("executing", "Start state and scene validated; explicit execution attempt admitted");
            const bool success = exec.executeSolution(plan->solution);
            gate_.finishExecution(success);
            if (success)
            {
                if (repeated) repeat_gate_.finish(true);
                else repeat_gate_.retainSuccessful(plan_id);
                completed_plan_ = std::move(plan);
            }
            else repeat_gate_.finish(false);
            report(success ? "succeeded" : "failed", success
                ? "Task completed. Return robot and scene to the recorded start before an explicit repeat"
                : "Execution stopped; inspect robot and scene before planning again", success);
        }
        catch (const std::exception &error)
        {
            gate_.finishExecution(false);
            repeat_gate_.finish(false);
            completed_plan_.reset();
            report("failed", std::string("Execution attempt stopped; request a new plan: ") + error.what());
        }
    }
''' + task[end:]
    task = replace(task, '        prepared->mode = active_task_mode_;', '        prepared->mode = active_task_mode_;\n        prepared->plan_id = newPlanId();')
    task = replace(task, '        publishExecutionStatus("ready", "A successful plan is available for one execution attempt");', '''        if (active_task_mode_ == TaskMode::TcpWaypoints)
            publishTcpExecutionStatus("ready", "A successful plan is available for one execution attempt", active_tcp_request_id_, prepared_plan_->plan_id);
        else
            publishExecutionStatus("ready", "A successful plan is available for one execution attempt", prepared_plan_->plan_id);''')
    task = replace(task, '        oss << "],\\"message\\":\\"" << jsonEscape(message) << "\\"";', '''        oss << "],\\"message\\":\\"" << jsonEscape(message) << "\\"";
        if (prepared_plan_)
            oss << ",\\"plan_id\\":\\"" << jsonEscape(prepared_plan_->plan_id) << "\\"";''')
    task = replace(task, '                // execution consumes the plan, so another request needs fresh planning.', '                // replay additionally requires the original scene, including the object, to be restored.')

    gate = replace(sources["include/ur16e_mtc_gpio_stage/plan_gate.hpp"], '  void finishExecution(bool success) noexcept', '''  // Only the separate identified Repeat endpoint may call this. Ordinary
  // beginExecution remains one-shot even when a completed solution is retained.
  bool beginRepeatExecution(bool has_successfully_completed_plan) noexcept
  {
    if (state_ != State::Idle || !has_successfully_completed_plan) return false;
    state_ = State::Executing;
    return true;
  }

  void finishExecution(bool success) noexcept''')
    cmake = replace(sources["CMakeLists.txt"], '  ament_add_gtest(test_plan_validation test/test_plan_validation.cpp)', '''  add_executable(test_repeat_execution test/test_repeat_execution.cpp)
  target_include_directories(test_repeat_execution PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(test_repeat_execution PRIVATE cxx_std_17)
  add_test(NAME repeat_execution COMMAND test_repeat_execution)

  ament_add_gtest(test_plan_validation test/test_plan_validation.cpp)''')
    generated = {"src/test_task.cpp": task, "include/ur16e_mtc_gpio_stage/plan_gate.hpp": gate, "CMakeLists.txt": cmake}
    baseline = PROJECT / "Library/RepeatExecutionValidation/baseline"
    isolated = PROJECT / "Library/RepeatExecutionValidation/source"
    for name, content in generated.items():
        path = ROOT / PACKAGE / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
        old = baseline / name
        old.parent.mkdir(parents=True, exist_ok=True)
        old.write_text(sources[name], encoding="utf-8")
    shutil.copytree(ROS / PACKAGE, isolated, dirs_exist_ok=True)
    shutil.copytree(ROOT / PACKAGE, isolated, dirs_exist_ok=True)
    patch = []
    paths = list(generated) + ["include/ur16e_mtc_gpio_stage/repeat_execution.hpp", "test/test_repeat_execution.cpp"]
    manifest = []
    for name in paths:
        current = ROOT / PACKAGE / name
        original = ROS / PACKAGE / name
        previous = original.read_text(encoding="utf-8-sig") if original.is_file() else ""
        new = current.read_text(encoding="utf-8-sig")
        patch.extend(difflib.unified_diff(previous.splitlines(True), new.splitlines(True),
                     fromfile="a/" + (PACKAGE / name).as_posix() if previous else "/dev/null",
                     tofile="b/" + (PACKAGE / name).as_posix()))
        manifest.append({"path": (PACKAGE / name).as_posix(), "beforeSha256": hashlib.sha256(original.read_bytes()).hexdigest() if original.is_file() else None,
                         "afterSha256": hashlib.sha256(current.read_bytes()).hexdigest()})
    (ROOT / "repeat-reviewed-plan.patch").write_text("".join(patch), encoding="utf-8")
    (ROOT / "manifest.json").write_text(json.dumps({"sourceRepository": str(ROS), "changes": manifest,
         "deployment": False, "hardware": False}, indent=2) + "\n", encoding="utf-8")
    print("Prepared backend repeat support and isolated source at", isolated)


if __name__ == "__main__":
    prepare()
