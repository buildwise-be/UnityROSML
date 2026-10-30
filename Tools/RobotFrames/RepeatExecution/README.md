# Repeat a reviewed robot plan

The accompanying patch implements `/repeat_reviewed_plan` in the local ROS
source. It is an additional explicitly identified action; the original
`execute_task` Bool and `execute_tcp_plan` request-ID action remain single-use.
No live stack is rebuilt, restarted, or operated by preparing/applying this patch.

## Wire contract

Request topic: `/repeat_reviewed_plan`, `std_msgs/String`:

```json
{"plan_id":"opaque-plan-token","attempt_id":"fresh-guid-for-this-click"}
```

Every successful prepared plan receives a session-unique opaque `plan_id`.
The plan token appears in planning and ready status messages. TCP's existing
`request_id` remains unchanged and is a separate field. Original execution
statuses carry an empty `attempt_id`.

The corresponding `/mtc/execution_status` or `/mtc/tcp_execution_status` JSON
contains `plan_id`, `attempt_id`, and `repeatable`. Only confirmed successful
execution advertises `repeatable:true` and retains the immutable prepared
solution. Repeated attempts echo both exact identifiers; clients must match both.
An unknown token's rejection is emitted on both execution-status topics because
it has no retained mode. Such rejection never modifies another retained plan.

An attempt GUID is remembered before preflight. Duplicate repeat callbacks are
silent so they cannot replace the actual execution's outcome with a stale
terminal status. A busy repeat callback also performs no operation and publishes
no terminal status, since it can be a duplicate of the in-flight attempt.
Clients must retain their acknowledgement/outcome wait and must never retry an
unknown outcome automatically. There is no automatic robot return motion.

Each explicit repeat runs the original full preflight again: unchanged
robot/tool descriptions and capability; full scene/world/attachments, collision
permissions, octomap, referenced transforms, padding and scaling; and current
joint feedback. Existing startup defaults remain 0.01 rad per joint and at most
one second since feedback receipt. Bounded-joint differences are not wrapped
modulo 2π. All preflight or execution failures consume repeat eligibility.

Returning the arm to its initial joints is necessary but does not restore a
moved object or planning scene. In particular, MTC pick/place's existing return
stage returns the arm but leaves the object at the place pose. Its original
scene must also match before the same solution can execute again. The endpoint
does not silently rewrite/reset objects, detach geometry, release vacuum, or
relax scene validation. A rejected attempt requires a new plan.

New planning, scene setup, model/tool invalidation, or failure discards the
retained successful solution. The attempt-ID set never evicts IDs for a current
plan; after 1,024 attempts it refuses further attempts until a new plan is made.

## Files and offline validation

`prepare_backend.py` reads the current local ROS sources, preserves their
existing precision fixes, produces this complete source overlay and
`repeat-reviewed-plan.patch`, and prepares an isolated source tree under
`Library/RepeatExecutionValidation`. `apply_backend.py` checks every recorded
original hash before copying only these five source/test files into that local
checkout, then appends the new API and local validation state to its docs.

The added C++ parser/admission suite passes 37 checks with
`-Wall -Wextra -Werror -pedantic`, and the original plan-gate suite passes 10/10.
The complete MTC package and executable compile in the cached local ROS image
with Docker networking disabled. Ten of eleven CTest targets pass. The remaining
failure is the unchanged previously documented scene-effects assertion at
`test_scene_effects.cpp:166`; all 14 plan-validation cases and the new repeat
target pass. No ROS application or robot operation was started.

Logs and XML stay under `Library/RepeatExecutionValidation/results`; compact
source hashes, limits, and outcomes are recorded in `validation_summary.json`.
Backend deployment is separate from this local source implementation.
