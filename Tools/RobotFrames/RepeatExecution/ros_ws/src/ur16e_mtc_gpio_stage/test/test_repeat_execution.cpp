#include "ur16e_mtc_gpio_stage/plan_gate.hpp"
#include "ur16e_mtc_gpio_stage/repeat_execution.hpp"
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

using ur16e_mtc_gpio_stage::PlanGate;
using ur16e_mtc_gpio_stage::RepeatExecutionGate;
using ur16e_mtc_gpio_stage::RepeatExecutionRequest;
using ur16e_mtc_gpio_stage::parseRepeatExecutionRequest;

void require(bool value, const std::string& message) { if (!value) throw std::runtime_error(message); }

int main()
{
  int checks = 0;
  const auto check = [&](bool value, const std::string& message) { require(value, message); ++checks; };
  RepeatExecutionRequest request;
  check(parseRepeatExecutionRequest("{\"plan_id\":\"plan-123\",\"attempt_id\":\"74d20825-5044-4a99-963e-8d16a3c2be5f\"}", request), "Parse production GUID request");
  check(parseRepeatExecutionRequest(" { \"attempt_id\" : \"attempt-1\", \"plan_id\": \"plan-1\" } ", request), "Accept field order and JSON whitespace");
  for (const std::string malformed : {"", "{}", "[]", "null", "{\"plan_id\":\"p\"}",
       "{\"plan_id\":\"p\",\"plan_id\":\"q\"}", "{\"plan_id\":\"p\",\"attempt_id\":\"a\",\"extra\":\"x\"}",
       "{\"plan_id\":\"p\",\"attempt_id\":true}", "{\"plan_id\":\"p\",\"attempt_id\":\"\"}",
       "{\"plan_id\":\"p\",\"attempt_id\":\"a\\n\"}", "{\"plan_id\":\"p\",\"attempt_id\":\"a\"}x"})
    check(!parseRepeatExecutionRequest(malformed, request), "Reject malformed JSON: " + malformed);
  check(!parseRepeatExecutionRequest(std::string(513, ' '), request), "Bound parser input");
  RepeatExecutionGate repeat;
  const RepeatExecutionRequest first{"plan-1", "attempt-1"};
  check(!repeat.begin(first).empty(), "No completed plan cannot repeat");
  repeat.retainSuccessful("plan-1");
  check(repeat.repeatable(), "Confirmed success advertises repeat");
  check(!repeat.begin({"other-plan", "attempt-1"}).empty() && repeat.repeatable(), "Wrong plan does not consume retained completion");
  check(repeat.begin(first).empty() && !repeat.repeatable(), "Explicit new attempt consumes eligibility before hardware");
  check(repeat.isDuplicate(first), "Record attempt before hardware");
  check(!repeat.begin(first).empty(), "In-flight duplicate cannot execute");
  repeat.finish(true);
  check(repeat.repeatable() && repeat.isDuplicate(first), "Success retains permanent deduplication history");
  check(!repeat.begin(first).empty(), "Delayed duplicate after return to start cannot execute");
  check(repeat.begin({"plan-1", "attempt-2"}).empty(), "New explicit attempt can repeat the same solution");
  repeat.finish(false);
  check(!repeat.repeatable() && !repeat.begin({"plan-1", "attempt-3"}).empty(), "Rejected/failed attempt cannot retry automatically");
  repeat.retainSuccessful("plan-2");
  check(!repeat.isDuplicate(first) && !repeat.begin(first).empty(), "Replacement plan rejects old IDs");
  repeat.invalidate();
  check(!repeat.repeatable() && !repeat.begin({"plan-2", "attempt-4"}).empty(), "Scene/plan invalidation revokes repeat");
  repeat.retainSuccessful("plan-limit");
  for (int i = 0; i < 1024; ++i) { require(repeat.begin({"plan-limit", "attempt-" + std::to_string(i)}).empty(), "Admission below bound"); repeat.finish(true); }
  check(!repeat.begin({"plan-limit", "attempt-overflow"}).empty(), "Attempt capacity fails closed without evicting tokens");
  check(repeat.isDuplicate({"plan-limit", "attempt-0"}), "Oldest duplicate remains remembered at capacity");

  PlanGate gate;
  check(!gate.beginExecution(true), "Legacy Execute cannot use idle cached storage");
  check(!gate.beginRepeatExecution(false), "Repeat requires retained successful solution");
  check(gate.beginPlanning(), "Admit first plan");
  gate.finishPlanning(true);
  check(!gate.beginRepeatExecution(true), "Unexecuted plan cannot enter repeat from Planned");
  check(gate.beginExecution(true), "First Execute stays one-shot");
  gate.finishExecution(true);
  check(!gate.beginExecution(true), "Original Execute remains consumed after success");
  check(gate.beginRepeatExecution(true), "Explicit completed-plan repeat admits from Idle");
  check(!gate.beginExecution(true) && !gate.beginRepeatExecution(true) && !gate.beginPlanning(), "Repeat serializes against all commands");
  gate.finishExecution(false);
  check(!gate.beginRepeatExecution(true), "Failure faults repeat admission");
  std::cout << checks << " repeat protocol/admission checks passed; no ROS runtime or hardware calls\n";
  return EXIT_SUCCESS;
}
