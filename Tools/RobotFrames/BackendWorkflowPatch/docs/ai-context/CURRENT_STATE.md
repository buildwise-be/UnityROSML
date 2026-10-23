# Current State

- Owner-requested contact workflow and execution-validator correction are
  implemented locally (2026-10-22): `pick_place_tcp_target` has an explicit
  tool-contact/+Z contract and independent twist samples at pick/place. The
  scene validator accepts MoveIt's shared target frame and excludes unused
  controller-feedback transforms from collision-scene comparison. Offline
  verification is recorded in [build/testing](../wiki/BUILD_AND_TESTING.md):
  the actual new node produced contact and exact-TCP preview plans against
  local mocked scene services in a network-disabled container.
  No Nordbo deployment, real-stack restart or hardware command was performed.

- Owner approved applying the exact TCP A/B plan/review patch, updating API/project notes and offline build/test on 2026-10-22. The MTC node now has separate plan-only/correlated preview and reviewed-ID execution interfaces; tile behavior remains separate. Applied-source full package compilation and focused tests passed, with one unchanged scene-effect assertion reproduced on the baseline. No deployment, application launch or hardware operation was authorized or performed. See [API](../wiki/API.md#plan-and-review-exact-tcp-ab-targets) and [verification](../wiki/BUILD_AND_TESTING.md#2026-10-22-exact-tcp-ab-plan-and-review).

- Owner authorized committing the WSL compatibility fixes and synchronizing/pushing both checkouts on `codex/architecture-review`. Both the Safe&Light integration and compatibility fixes are retained in the combined history.
- Combined-history verification passed actual control-wrapper generation and all four model scenarios (eight URDFs/four SRDFs), syntax, links and framework checks. Full combined ROS build/launch/MoveIt tests and hardware operation remain unverified; see [dated verification](../wiki/BUILD_AND_TESTING.md#2026-10-22-combined-history-verification).

- Latest task (2026-10-22): registered `kenos_sl` throughout startup, description/SRDF generation and MTC vacuum behavior, including the dedicated `urmir_rsp_kenos_sl.launch.py` driver wrapper. Five separate mounting/TCP arguments are forwarded consistently to RSP, MoveGroup and MTC. The owner confirmed the existing Kenos DO3/on/off/release settings. The STEP-derived 34 mm face-centre offset is retained; mounting height/radius and clocking remain approximate. Verification and remaining runtime limits are recorded in [build/testing](../wiki/BUILD_AND_TESTING.md).
- Integration verification: seven portable C++ capability cases, six standalone Xacro cases and four combined model scenarios passed (eight control/no-control URDFs plus four SRDFs), along with syntax/shell/documentation/framework checks. Eleven real ROS launch cases and ten MoveIt model cases are registered but unrun locally; full ROS build, runtime, physical calibration and hardware IO remain unverified.
- On 2026-10-21, migrated the two shell motion examples to the explicit TCP topics at the owner's request. Bash syntax, stubbed publication/payload checks and Git whitespace checks passed; ROS planning/execution remain unverified. No hardware operation or application build was performed.

## UR driver compatibility correction (2026-10-21)

- Owner approved conditional `ur_type` forwarding in the local WSL checkout. The control package now generates its installed Xacro wrapper at CMake configuration time from the selected driver's actual `ur_ros2_control` signature: it forwards the argument when declared and omits it otherwise. The installed filename and launch interfaces remain unchanged; rebuild the control package after changing the driver.
- Verification passed: 16 generator cases and seven existing model-consistency tests, plus eight complete Xacro expansions using the exact upstream 3.8.0 and installed 3.9.0 macros, `none`/`kenos`, and mock/real hardware descriptions. These checks start no ROS nodes and establish no hardware runtime compatibility; the older macro was exercised with the other current dependencies.
- The application image rebuilt successfully (58 packages), reusing the existing base image. Container `urmir-jazzy` was recreated with image `sha256:60be90bce31c757d3371129cd7d1ce1dc8d4cf17c3124c5e013bf880fd8b9993`; its installed Kenos mock model expands successfully. Full bringup, RViz, robot communication, and unrelated ROS test suites remain unverified.
- These changes were initially isolated in WSL; committed as `6e60cda` on 2026-10-22 and combined with the Windows Safe&Light work at the owner's request. Nordbo deployment was not changed. See [wrapper generation and verification](../wiki/BUILD_AND_TESTING.md#ur-control-wrapper-generation).

## Local WSL Docker build (2026-10-21)

- Owner approved backporting the MTC pybind11 ordering fix and rebuilding both images in `/home/aca/UR16UnityControl` under WSL Ubuntu 24.04. The Windows checkout and Nordbo deployment were not changed.
- Base image `urmir-jazzy-base:latest` built successfully: 48 packages completed. Application image `urmir-jazzy:latest` built successfully: 58 packages completed, including the commander and MTC task node. The existing MTC tag and Python bindings were retained.
- Offline smoke checks passed: MTC Python imports in both images, application package discovery, and shared-library resolution for both command executables. No ROS nodes, GUI, mock bringup, or hardware were started during these checks; later focused model/generator test results are recorded above.
- Existing nonblocking build messages remain, including the unresolved `Eigen3` rosdep key and a prompt setup call to undefined `parse_git_branch`. Eigen development packages are present and application compilation succeeded. See [build verification](../wiki/BUILD_AND_TESTING.md#2026-10-21-local-docker-build-verification).
- The local WSL changes were committed as `6e60cda` on 2026-10-22. No new Issue, PR or Nordbo deployment accompanied the October 21 builds.

## Previous architecture-review handoff (2026-09-29)

- Mission at that handoff (2026-09-29): commit and push the three remaining original P1 corrections at the owner's request.
- User requested the Buildwise development guide before the review. Framework 2.0.1 was initialized from upstream commit `9307d3003afb9be6389de4ed1a8cbdbf518d77e5`.
- Review branch: `codex/architecture-review`; initial working tree on `main` was clean.
- Authorized scope: the fifth P1 capability gate, startup tool/model validation, regression coverage and related documentation. Preserve the third and fourth P1 work already in the working tree. The first two P1 fixes were committed and pushed as `b425326` and `28fda0c`; wider architecture changes and hardware operation are outside this implementation.
- Functional maturity and deployment status have not been confirmed by the owner.
- Review completed: [architecture assessment and proposals](ARCHITECTURE_REVIEW.md), with current technical behavior mapped in [the wiki](../wiki/INDEX.md).
- The owner confirmed tool selection is performed at startup. Runtime tool changing is outside the proposed architecture.
- Implemented in the working tree: explicit admission states, callback-level busy rejection, a frozen successful plan, invalidation on replanning/scene changes, consumption before execution attempts, complete construction/init/plan exception boundaries, start-scene/joint preflight and additive execution status. Preview invalidation removes stale retained trajectories.
- Verification: ten new portable PlanGate tests passed using g++ 13.3.0 with warnings as errors and checks enabled under `NDEBUG`. Five ROS-message validation tests were added but remain unrun, as does the full task-node build and ROS callback/service integration. See [build/testing](../wiki/BUILD_AND_TESTING.md).
- Final static checks (2026-09-14) passed: ten Python AST parses, 25 XML parses, 167 relative documentation links, Git whitespace checks and Buildwise full-profile validation. No lint target is registered.
- Prior first-P1 model checks passed six isolated Xacro scenarios. That does not establish the deployed ROS dependency versions or runtime behavior.
- Fourth P1 implementation: new explicit TCP topics validate complete requests and resolve stamped frames once before motion; old wrist topics require an explicit startup compatibility opt-in. Sequential moves stop on the first planning/execution failure and publish indexed status. As of 2026-10-21, the two shell examples use explicit TCP goals with zero timestamps and no legacy opt-in; their sample numbers no longer describe wrist goals.
- Fourth P1 verification: eight portable sequencing cases passed; nine ROS-message/tf2 cases were added but remain unrun, as does the full commander build. Unity callers and runtime TF/action behavior remain unverified.
- Fifth P1 implementation: MTC resolves an immutable mounted-tool definition at startup and validates it against the loaded model before accepting commands. `none` has no vacuum binding; pick/place and execution requests invalidate readiness and are rejected before task scene changes, planning or executor creation. Kenos contact links and IO settings come from its definition. Tool and TCP parameters are read-only after startup.
- Fifth P1 verification: six portable capability cases passed; eight actual MoveIt-model tests and six rclcpp startup-parameter tests were added but remain unrun. The existing ROS launch/model suite also checks behavioral tool selection; its execution and the full MTC build remain pending in a ROS installation.
- All five original P1 findings are addressed in source. Full ROS/runtime validation is still outstanding; neither commander provides shared robot ownership or command/plan IDs. Broader tool/deployment configuration consolidation remains proposed work.
- Full ROS suites, ROS package build, Docker, mock runtime and hardware remain unrun: this environment has no ROS installation. The separate newest-driver Xacro incompatibility remains recorded in [known issues](KNOWN_ISSUES.md).
- Delivery branch: `codex/architecture-review`. The fourth P1 code/tests are committed as `aedbb26`; the third and fifth P1 code/tests are committed together as `a128002`. The owner authorized pushing these commits and their documentation on 2026-09-29. No GitHub Issue or PR has been created.
- Nordbo deployment has not been performed in this session. The read-only SSH check on 2026-09-29 timed out; its installed checkout/image remains unconfirmed. Pulling the branch and rebuilding the application image are still required to deploy these changes.
