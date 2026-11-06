# Pick Descent Compatibility and Item Outline Implementation Plan

**Goal:** Make selected descent speed usable in the existing ordinary ROS pick mode and show the accepted item's fitted outline in Unity.

**Architecture:** Retain separate ordinary and guarded execution paths; retime the ordinary Cartesian approach using the existing TCP trajectory timing helper. Extend the shared contour renderer with an independent accepted-item source driven by the existing controller/capture lifecycle.

**Tech stack:** Unity 6000/C#, URP, ROS 2 Jazzy/C++, MoveIt Task Constructor.

**Spec:** ../specs/2026-11-06-pick-descent-and-item-outline-design.md

## Global constraints

- Do not enable or commission guarded contact automatically.
- Keep 5/10/20 mm/s and immutable per-plan execution/repeat settings.
- Preserve legacy requests and the owner's unrelated ProjectSettings edit.
- No production motion, IO, controller switching, or automatic stack restart.

## Review focus

- Disabled guarded mode must produce a physically timed ordinary descent, not just accept the request.
- Prepared-plan metadata must survive both modes and explicit repeats.
- Independent item outlines must work when robot contours are off and restore filled fallback when unavailable.
- Invalid context must not leave stale visible item geometry.
- Recapture failure/cancellation must preserve the previous accepted pose and dimensions.

## Task 1: Ordinary ROS descent

- [x] Add/run a failing disabled-guard task-setting test and ordinary speed trajectory test.
- [x] Retain requested speed without changing the startup enable flag; use a per-task Cartesian planner/timing path for the ordinary pick approach.
- [x] Freeze request metadata in prepared plans outside the guarded-only binding block.
- [x] Build/test in a network-disabled disposable container; verify actual-node planning and synthetic Execute/repeat at selected speeds. Recheck guarded behavior and invalid requests.
- [x] Refresh backend context/wiki and review the diff.

## Task 2: Accepted-item outline

- [x] Add/run failing tests for accepted full-size geometry and independent contour draws, cancellation, invalidation, and cleanup.
- [x] Register the accepted item as an independent source in RobotContourDisplay/RobotContourRendererFeature; retain per-camera fallback and original renderer flags.
- [x] Drive a fitted mesh visual from PickPlaceController/PickSurfaceCapture with correct local dimensions and body centre, matching pick-target visibility and validity.
- [x] Fix the observed grazing-plane depth-edge false positive in the shared contour shader; retain real depth jumps and orthographic behavior.
- [x] Run Unity regression checks and render accepted tile/brick using the actual contour feature.

## Task 3: Handoff

- [x] Review both changes, run runtime compilation and final targeted regression checks, and record evidence.
- [x] Commit/push source fixes and update the existing draft PR summaries.
- [x] Report root cause, resulting behavior, tests, and the exact deployment status without implying hardware verification.

