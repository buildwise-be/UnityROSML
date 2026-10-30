#pragma once

namespace ur16e_mtc_gpio_stage
{

// Admission state only: the task/solution storage remains owned by the caller.
// The caller must serialize access and hold that protection through planning or
// execution, including checking solution storage and using the admitted plan.
class PlanGate
{
public:
  enum class State { Idle, Planning, Planned, Executing, Faulted };

  State state() const noexcept { return state_; }

  bool isBusy() const noexcept
  {
    return state_ == State::Planning || state_ == State::Executing;
  }

  bool beginPlanning() noexcept
  {
    if (isBusy())
      return false;
    // Starting a replacement plan immediately removes the old plan's eligibility.
    state_ = State::Planning;
    return true;
  }

  void finishPlanning(bool success_and_solution) noexcept
  {
    if (state_ == State::Planning)
      state_ = success_and_solution ? State::Planned : State::Faulted;
  }

  bool beginExecution(bool has_solution) noexcept
  {
    if (state_ != State::Planned)
      return false;
    if (!has_solution)
    {
      // The saved task no longer satisfies the invariant established by planning.
      state_ = State::Faulted;
      return false;
    }
    // Consume eligibility before the caller dereferences or operates the plan.
    state_ = State::Executing;
    return true;
  }

  // Only the separate identified Repeat endpoint may call this. Ordinary
  // beginExecution remains one-shot even when a completed solution is retained.
  bool beginRepeatExecution(bool has_successfully_completed_plan) noexcept
  {
    if (state_ != State::Idle || !has_successfully_completed_plan) return false;
    state_ = State::Executing;
    return true;
  }

  void finishExecution(bool success) noexcept
  {
    if (state_ == State::Executing)
      state_ = success ? State::Idle : State::Faulted;
  }

  void invalidate() noexcept
  {
    if (!isBusy())
      state_ = State::Idle;
  }

private:
  State state_ = State::Idle;
};

}  // namespace ur16e_mtc_gpio_stage
