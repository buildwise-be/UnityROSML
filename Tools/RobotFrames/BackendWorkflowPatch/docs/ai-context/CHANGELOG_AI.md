# AI-assisted Changelog

## 2026-10-22

- Corrected execution transform validation to use MoveIt source frame keys
  and compare only collision-referenced transforms. Added the separately
  advertised tool-contact pick/place endpoint with rotated object offsets and
  independent pick/place twist sampling; preserved the old surface API. Added
  source/target and encoder-jitter regressions plus contact geometry tests.
  Offline package build/test and actual-node mock-scene smoke results are
  in build/testing: contact and subsequent exact-TCP plans produced previews,
  with no Execute request, deployment or hardware operation.

- Applied the owner-approved exact TCP A/B MTC plan/review change with separate correlated outputs and reviewed-ID single-use execution. Preserved tile vacuum behavior and prohibited IO for TCP plans. Updated API/task/build notes and project memory. Full applied-source package compilation and focused tests passed; the sole legacy scene-effect assertion also fails on the untouched baseline. No deployment, application launch or hardware operation was performed.

- Committed the WSL MTC build and UR driver compatibility fixes as `6e60cda` and combined them with the Windows Safe&Light history at the owner's request. Resolved documentation conflicts by retaining both implementations and their dated verification records.

- Integrated `kenos_sl` into shell/launch tool choices, a dedicated RSP wrapper, combined Xacro and SRDF, and MTC capability/model validation. All model consumers receive the same five Safe&Light mounting/TCP arguments; FlexiGrip TCP defaults stay separate. The owner confirmed the shared DO3 vacuum binding.
- Expanded the real ROS launch/model suite to eleven cases, portable tool-definition coverage to seven cases, and MoveIt model coverage to ten cases. Verification status and remaining ROS build/runtime limits are recorded in [build/testing](../wiki/BUILD_AND_TESTING.md). No ROS nodes, containers or hardware were started.
- Integration verification passed: seven portable capability cases, six standalone Xacro cases, eight combined control/no-control URDFs and four SRDFs using pinned official dependencies, plus shell/syntax/documentation/framework checks. Full ROS launch tests, MoveIt model tests and package build remain unrun locally.

- Added the owner-approved reusable `kenos_sl` description, using primitive visuals/collisions, a fixed flange attachment and the supplied KSL STEP's 34 mm eccentric face centre. Mounting height, radius and yaw are exposed for the actual changer/adapter; estimates are identified in source comments and project memory.
- Added six actual-Xacro model regression cases and package test registration. They check default and rotated TCP transforms, calibration overrides, prefixed composition and primitive collision coverage, including the ejector and air fitting. All passed with Xacro 2.1.1 and PyYAML 6.0.3 loaded from an isolated temporary directory.
- Updated tool documentation and verification records. The approved scope creates the reusable model; startup tool selection and vacuum behavior integration remain separate. No application/ROS package build, ROS runtime or hardware operation was performed.

## 2026-10-21

- Owner requested replacing the two legacy shell publishers with examples for `/tcp_pose_target` and `/tcp_pose_array_target`. Both now publish explicit TCP goals once with zero timestamps; the sample values describe the TCP rather than preserving the former wrist configurations.
- Updated README usage, API and verification documentation, and current project memory. Bash syntax and stubbed publication checks passed, including payload frames, timestamps, pose counts, finite coordinates and unit quaternions. Git whitespace checks passed. No ROS nodes, containers or hardware ran; application build and ROS integration checks remain unrun.

## 2026-09-29

- Owner authorized committing and pushing the remaining P1 corrections. Committed basic frame/TCP handling as `aedbb26` and the intertwined plan-admission/tool-capability changes as `a128002`, with their regression tests.
- Packaged the accompanying API, migration, verification and project-status documentation for the same branch delivery. Prior verification results and unrun ROS build/tests remain explicitly recorded; no new robot or container operation was performed.
- Nordbo's read-only SSH inspection timed out. GitHub delivery does not establish deployment there; the application image still needs rebuilding from the updated checkout.

## 2026-09-14

- Implemented the fifth P1: immutable mounted-tool capability definitions, startup model validation, and explicit unsupported vacuum-task rejection before scene/planning/IO work. Scene setup and basic TCP motion remain available with no tool.
- MTC launch now passes its selected tool as a behavior parameter; tool and IK frame are read-only at startup. Validated contact links, service, digital output and release delay come from the Kenos definition. Existing launch/model tests also check the behavior parameter matches the model selection.
- All five original P1 findings were addressed in source; ROS and physical validation remain outstanding. Commit delivery is recorded in the 2026-09-29 entry above.
- Six portable capability tests passed under `NDEBUG` with warnings as errors. Eight MoveIt-model and six rclcpp startup-parameter tests were added but not run; the expanded real launch/model suite and full MTC build remain unrun without ROS. No ROS nodes, containers or hardware ran.
- Final fifth-P1 static checks passed: ten Python AST parses, 25 XML parses, 167 relative documentation links, Git whitespace checks and Buildwise full-profile validation.

- Implemented the fourth P1: new stamped TCP inputs, complete-request validation, one TF snapshot per request, explicit model-validated TCP targeting and fail-fast sequential motion with indexed status.
- Added an opt-in compatibility adapter for legacy wrist topics, exposed through main bringup. Existing examples retain their original wrist coordinates; documented the Unity migration and standalone wrist demo.
- Eight portable command-sequencing cases passed under `NDEBUG` with warnings as errors. Nine ROS-message/tf2 tests were added but not run; full ROS compilation and callback/TF/action integration remain unverified locally. No ROS nodes, containers or hardware ran.
- Preserved the prior third P1 work alongside the fourth fix for subsequent commit delivery.
- Final Python/XML syntax, 154 relative documentation links, whitespace and Buildwise full-profile checks passed. No lint target is registered.

## 2026-09-10

- Implemented the third P1: explicit plan admission, frozen successful solutions, invalidation/consumption, callback-level busy rejection, and exception boundaries covering construction, initialization and planning.
- Added preflight checks for current scene/joints and recent feedback, bounded scene requests, model-source change detection, empty preview publication on invalidation, and additive `/mtc/execution_status` messages.
- Ten portable PlanGate tests passed with g++ 13.3.0, `NDEBUG` and warnings as errors. Five ROS-message validator tests were registered but not run; the full ROS task-node build and service/callback integration remain unverified locally. Subsequent commit delivery is recorded above.

- Implemented the owner-authorized second P1 correction: apply each MTC scene effect after its operation succeeds, including empty and IO-only stages, and require acknowledgement before advancing.
- Preserve full/diff scene semantics, attached bodies, world placement and collision changes; clear planned joint-state fields so execution does not overwrite measured joints.
- Added bounded scene-service calls and failure handling that stops later work without retries, rollback or automatic vacuum release. Documented unknown timeout outcomes and manual recovery limits.
- Seven portable C++ regression cases passed using the production sequencing helper, compiled with g++ 13.3.0 and warnings as errors. Added separate actual MoveIt scene tests for execution in a ROS environment; these and the full ROS package build remain unrun locally.
- Updated the README, technical wiki and project memory. No ROS nodes, GPIO calls, containers or hardware operations were started. The owner authorized committing and pushing this correction.

## 2026-09-09

- Implemented the owner-authorized first P1 correction: explicitly pass selected tool, UR type and the same arm-calibration file to RSP, MoveGroup and MTC from the main bringup launch.
- Added standalone planning defaults, calibration package dependencies, and seven registered ROS launch/Xacro regression cases covering both tools, calibration overrides and an arm-type override.
- Six independent source-derived Xacro scenarios passed with official UR driver tag `3.2.0`; physical models agree and the old planner defaults produce detectable mismatches. Full ROS launch regression execution remains pending because ROS is unavailable locally.
- Recorded a separate pre-existing incompatibility with the newest Jazzy UR driver control macro. Updated technical wiki, README and project memory with the fix scope, verification evidence and remaining limitations.
- Final Python/XML syntax, documentation-link, Git whitespace and Buildwise full-profile checks passed. No registered lint checks were available.
- No ROS nodes, Docker containers or robot hardware were started; no C++ code changed.

## 2026-09-07

- Initialized Buildwise development framework 2.0.1 for the requested architecture review.
- Created project memory based on the current user request; historical intent and deployment status remain unconfirmed.
- Completed independent reviews of tool/model composition, MTC execution and build/deployment, plus basic commands, interfaces, environment and overall bringup.
- Added six technical wiki pages and an architecture assessment with prioritized proposals and validation criteria.
- Recorded the owner's startup-only tool selection requirement.
- Passed static Python/XML checks, framework validation and documentation link checks. No application implementation changed; no runtime, build, hardware or Unity validation was performed.
