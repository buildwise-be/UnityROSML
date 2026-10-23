# Build, deployment, and testing

This repository is a ROS 2 Jazzy workspace with ten local `ament_cmake` packages. Its documented deployment is a Linux Docker container with X11 display forwarding. Unity/Magic Leap client sources are outside this checkout.

## Docker build

The build has two Dockerfiles:

| Layer | Inputs and behavior |
| --- | --- |
| [Base image](../../docker/Dockerfile.base) | Starts from `ros:jazzy-ros-base`, installs `ros-jazzy-ur` and utilities, copies the RViz Debian package, clones ROS-TCP-Endpoint/MoveIt/MTC, runs rosdep, then builds `/home/ros_ws`. |
| [Application image](../../docker/Dockerfile) | Starts from local `urmir-jazzy-base:latest`, copies the local `ros_ws` into the same `/home/ros_ws`, runs rosdep, then runs `colcon build` in that workspace. |

The base retains third-party sources, build artifacts, and installed packages. The application build invokes colcon without a package-selection filter, so inherited third-party packages remain discoverable. Unchanged artifacts can be reused incrementally; this is not a separate dependency installation plus an isolated application workspace.

[`.dockerignore`](../../.dockerignore) excludes the checkout's `ros_ws/build/`, `ros_ws/install/`, and `ros_ws/log/`, preserving those directories inherited from the base image when application sources are copied.

The commands implemented by [run_docker.sh](../../docker/run_docker.sh) are:

```bash
# From the repository root: build images without starting the container.
./docker/run_docker.sh build

# Build and start the interactive container.
./docker/run_docker.sh

# Explicit base-image build, also documented in the Dockerfiles.
docker build -f docker/Dockerfile.base -t urmir-jazzy-base:latest .
```

The helper resolves its own directory, checks whether `urmir-jazzy-base:latest` exists locally, and builds it only when absent. It does not compare an existing base image with the current Dockerfile. Build-only mode skips display setup. Run mode executes `xhost +local:docker` when `DISPLAY` is set and then `docker compose ... up --build`.

The base build sets `CMAKE_BUILD_PARALLEL_LEVEL=2` and uses the sequential colcon executor with one parallel worker. The application inherits that environment variable but invokes colcon without the base's executor/worker arguments. Both build with Ninja and `CMAKE_BUILD_TYPE=Release`.

## Dependency sources and package declarations

| Dependency | Version selector recorded in [Dockerfile.base](../../docker/Dockerfile.base) |
| --- | --- |
| ROS base image | `ros:jazzy-ros-base` tag; no digest recorded |
| Universal Robots driver | Apt package `ros-jazzy-ur`; no exact Debian version recorded |
| ROS-TCP-Endpoint | Shallow clone selecting `ROS2v0.7.0` |
| MoveIt 2 | Shallow clone of moving branch `jazzy`; commit `e489b0483af155062df30573aea75e1c015a34ab` appears only as a known-working comment |
| MoveIt Task Constructor | Shallow clone selecting `ros2-0.1.4`; comment records commit `3cdf7f60fae2ca813de7b6616a654fef1bdb26cf`; local pybind11 dependency-order backport described below |
| RViz workaround | Vendored `ros-jazzy-rviz-common_14.1.11-1noble.20250520.201719_amd64.deb`, installed with `dpkg` and an apt dependency-repair fallback |

The Dockerfiles run `rosdep update` and `rosdep install --from-paths src --ignore-src -r -y`; no repository-level dependency lockfile or apt snapshot is tracked. The RViz file is explicitly `amd64`; the Docker configuration does not declare an alternate architecture or an apt hold for this package.

Local dependency declarations are in each package's `package.xml` and `CMakeLists.txt`. The bringup manifest lists control, MoveIt configuration, commander, environment, and ROS-TCP-Endpoint as runtime dependencies. Its launch file also includes `ur16e_mtc_gpio_stage`, which is declared as a test dependency for model checks but not as a runtime dependency. MoveIt configuration and MTC explicitly declare control as a runtime dependency for their calibration defaults. See [bringup manifest](../../ros_ws/src/ur16e_bringup/package.xml) and [bringup launch](../../ros_ws/src/ur16e_bringup/launch/ur16e_start.launch.py).

The MTC package's CMake requires `eigen3_cmake_module` and `Eigen3`; its manifest contains `<depend>Eigen3</depend>` and does not list `eigen3_cmake_module`. Direct `sensor_msgs` declarations were added with the valid-plan gate on 2026-09-10. A clean ROS dependency installation was not exercised. See [MTC manifest](../../ros_ws/src/ur16e_mtc_gpio_stage/package.xml), [CMake](../../ros_ws/src/ur16e_mtc_gpio_stage/CMakeLists.txt), and [source](../../ros_ws/src/ur16e_mtc_gpio_stage/src/test_task.cpp).

[`.gitmodules`](../../.gitmodules) contains two MTC sections pointing at an SSH URL for an ArnaudBBRI fork, including one section without a path. `git ls-tree -r HEAD` contains no `160000` Gitlink entries, and this checkout has no MTC source directory. The Docker build obtains upstream MTC independently; the `.gitmodules` entries do not record a dependency commit in the reviewed tree.

### UR control wrapper generation

[Control CMake](../../ros_ws/src/ur16e_mir250_control/CMakeLists.txt) runs
[generate_control_xacro.py](../../ros_ws/src/ur16e_mir250_control/scripts/generate_control_xacro.py)
at configure time using the driver's `urdf/ur.ros2_control.xacro` located relative
to `ur_robot_driver_DIR`. The generator reads the unique `ur_ros2_control` macro
and parses its declared parameter names with Xacro's `parse_macro_arg`.
It inserts `ur_type="$(arg ur_type)"` into the
[source template](../../ros_ws/src/ur16e_mir250_control/urdf/ur16e_mir250.urdf.xacro.in)
only if that signature accepts `ur_type`; older signatures receive no attribute.
Missing or ambiguous macros, an invalid template marker, or invalid generated
XML fail package configuration.

The generated file is installed under the unchanged name
`share/ur16e_mir250_control/urdf/ur16e_mir250.urdf.xacro`; the `.in` template is
excluded from installation. `CMAKE_CONFIGURE_DEPENDS` tracks the template,
generator, and inspected driver macro so the next build reconfigures when they
change. After a driver upgrade or overlay switch, rebuild the control package
and source the resulting workspace; the installed wrapper does not adapt at
runtime. In an already built, sourced workspace:

```bash
cd ros_ws
colcon build --packages-select ur16e_mir250_control --cmake-force-configure
source install/setup.bash
```

### MTC pybind11 dependency ordering

The base image applies [mtc-pybind11-order.patch](../../docker/patches/mtc-pybind11-order.patch)
after `rosdep install` and before `colcon build`. It backports the dependency-order
change from [upstream MTC commit 158908a](https://github.com/moveit/moveit_task_constructor/commit/158908aba9156afb3cd58aa2a8afbb8f9ffc2f13)
to the retained `ros2-0.1.4` tag: MTC loads its bundled pybind11 before
`find_package(py_binding_tools REQUIRED)` can load system pybind11. The unrelated
upstream deletion of an unused template is omitted. Python bindings remain enabled.

[Dockerfile.base](../../docker/Dockerfile.base) checks patch applicability with
`git apply --check` before applying it. Placing the patch after dependency
installation preserves the earlier Docker cache layers. The application image
inherits the patched MTC source and build artifacts from the base image; its
incremental `colcon build` uses that same patched source.

## Container and startup behavior

[Compose](../../docker/docker-compose.yaml) creates a container and hostname named `urmir-jazzy`, forwards ports `10000` and `50001-50004`, passes `DISPLAY` and `QT_X11_NO_MITSHM`, mounts `/tmp/.X11-unix`, enables privileged mode, and starts an interactive `/bin/bash`. It does not automatically launch ROS bringup. Workspace setup is appended to the image's interactive `.bashrc`.

The wrapper's documented launch commands are:

```bash
cd ros_ws
./run_bringup.sh -t kenos
./run_bringup.sh -t none
./run_bringup.sh -m
```

[run_bringup.sh](../../ros_ws/run_bringup.sh) invokes `ros2 launch ur16e_bringup ur16e_start.launch.py` with explicit robot/tool/hardware/RViz arguments. It assumes `ros2` and the workspace are already available in the shell; it does not source setup files itself.

The wrapper and direct launch have different defaults:

| Setting | Wrapper | Direct [ur16e_start.launch.py](../../ros_ws/src/ur16e_bringup/launch/ur16e_start.launch.py) |
| --- | --- | --- |
| Tool | `kenos` | `none` |
| Robot type | `ur16e` | `ur16e` |
| Robot IP | `192.168.125.101` | `192.168.125.101` |
| Mock hardware | `false` | `true` |
| MoveIt RViz | `true` | `true` |

The direct launch declares a ten-second `startup_delay` before starting MoveIt, the commander, the environment node, ROS-TCP-Endpoint, and the MTC launch. ROS-TCP-Endpoint is configured as `0.0.0.0:10000`. The wrapper accepts `none`, `kenos` and `kenos_sl` as tool choices and validates robot-type arguments before launching.

## Other build and configuration paths

[README.md](../../README.md) also documents manual apt installation, source clones, rosdep installation, and `colcon build`. These instructions differ from the Docker path: the manual MTC command selects branch `jazzy`, while the base Dockerfile selects `ros2-0.1.4` and comments that upstream has no `jazzy` branch. The older README Compose instructions start with `docker-compose build` without first creating the required base image. These paths were not executed during review.

The MoveIt package's [`.setup_assistant`](../../ros_ws/src/ur16e_mir250_moveit_config/.setup_assistant) records `config/ur16e_mir250.srdf`; the tracked configuration file is instead `config/ur16e_mir250.srdf.xacro`. The [standalone RViz](../../ros_ws/src/ur16e_mir250_moveit_config/launch/moveit_rviz.launch.py) and [setup assistant](../../ros_ws/src/ur16e_mir250_moveit_config/launch/setup_assistant.launch.py) launches use default `MoveItConfigsBuilder` discovery. The main [move_group launch](../../ros_ws/src/ur16e_mir250_moveit_config/launch/move_group.launch.py) explicitly supplies the Xacro semantic path.

[generate_urdf.sh](../../ros_ws/src/ur16e_mir250_description/scripts/generate_urdf.sh) expands the description without launching nodes and can bundle referenced meshes. It needs sourced ROS package discovery and `xacro`; bundle mode additionally invokes `ros2 pkg prefix --share`. Its default is `tool=kenos`, `use_control=false`. The description package's [CMake](../../ros_ws/src/ur16e_mir250_description/CMakeLists.txt) installs `urdf`, `include`, and `rviz`, but not this script.

## 2026-10-21 local Docker build verification

The owner-approved rebuild ran in `/home/aca/UR16UnityControl` on WSL Ubuntu
24.04 with Docker Engine 29.9.2 and Compose v5.6.0:

```bash
docker build --progress=plain -f docker/Dockerfile.base -t urmir-jazzy-base:latest .
./docker/run_docker.sh build
```

The base build checked/applied the patch successfully and completed all 48
packages, including MTC core, Python bindings, capabilities and visualization.
The application build reused those artifacts and completed all 58 packages,
including `ur16e_commander` and `ur16e_mtc_gpio_stage`.

| Image | Recorded local image ID (`sha256`) |
| --- | --- |
| `urmir-jazzy-base:latest` | `1871638572aedcf9c2e9a9eafcb330e752e2752100142e57ad5dce12cc38e1b8` |
| `urmir-jazzy:latest` | `7e28f1031ec4d16fe9af51e1c1c3f4c93541f61cb30f02efae0776fc68d3589c` |

Smoke checks in disposable containers with `--network none` passed: importing
`moveit.task_constructor.core` and `stages` in both images, discovering the four
application bringup/commander/MTC/MoveIt packages, and resolving shared libraries
for `ur16e_commander` and `test_task`. No ROS nodes or GUI were launched.
These checks supersede the earlier compilation limitations below; they do not
execute the ROS test suites, model expansion, mock bringup, or hardware paths.

Existing nonblocking messages include setuptools deprecations, a MoveIt test
package without an install target, a `tl_expected` deprecation, the unresolved
`Eigen3` rosdep key, and a shell-prompt reference to undefined `parse_git_branch`.
Both Eigen development packages are installed and CMake compilation succeeded;
this build does not establish clean rosdep metadata or runtime configuration.
Build logs were saved locally in `/tmp/urmir-base-build-20261021.log` and
`/tmp/urmir-app-build-20261021.log`.

## Test inventory and review verification

There is no tracked CI workflow. Since 2026-09-09, `ur16e_bringup` registers
[test_model_consistency.py](../../ros_ws/src/ur16e_bringup/test/test_model_consistency.py)
through `ament_cmake_pytest`. It contains eleven cases: all three tools with stored
or modified arm calibration, all five Safe&Light mounting/TCP overrides together
with arm calibration, isolation from FlexiGrip TCP arguments, standalone
MoveGroup/MTC defaults, and an explicit UR5e type/calibration override. It uses real launch substitutions and Xacro to compare
link geometry, joint definitions, TCP topology and calibration values; no ROS
nodes or driver processes are started. Driver metadata and existing differences
in safety-controller tags are excluded from physical-model equality.

The test checks the local forwarding boundary into the upstream driver and then
evaluates the selected local RSP wrapper directly. It does not execute the upstream
driver launch or reproduce its process/event lifecycle.

Since 2026-09-14, the existing cases also require the evaluated MTC node's `tool`
behavior parameter to match the selected URDF/SRDF tool, including standalone
default selection. This checks the real launch parameter boundary without
starting the task node. The previous seven-case suite passed in the 2026-10-21 application-image verification described below; the expanded eleven-case suite remains unrun.

Run in a built and sourced ROS Jazzy workspace, after installing declared test
dependencies:

```bash
cd ros_ws
source install/setup.bash
colcon test --packages-select ur16e_bringup --event-handlers console_direct+
colcon test-result --verbose
```

For a direct run against the sourced installed launch/configuration files:

```bash
python3 -m pytest src/ur16e_bringup/test/test_model_consistency.py -v
```

Missing ROS dependencies fail collection rather than silently skipping the tests.
Several manifests also declare lint test dependencies, but no local CMake file
registers those lint checks.

The older files named as tests are runtime programs or input publishers:

- [test_moveit.cpp](../../ros_ws/src/ur16e_commander/src/test_moveit.cpp) is built and installed as an executable. It plans and executes robot motions; it contains no test assertions and returns zero after normal completion.
- [test_send_pose.sh](../../ros_ws/src/ur16e_commander/src/test_send_pose.sh) and [test_send_poseArray.sh](../../ros_ws/src/ur16e_commander/src/test_send_poseArray.sh) publish command topics once; they do not assert resulting behavior.
- [test_task.cpp](../../ros_ws/src/ur16e_mtc_gpio_stage/src/test_task.cpp) is the operational MTC node launched by `mtc_commander.launch.py`, despite its test-oriented name.

During the 2026-09-07 review, Python AST parsing passed for nine launch Python files, and XML well-formedness parsing passed for 25 package/Xacro files. These checks do not resolve ROS imports, expand Xacro macros, validate model semantics, compile C++, or exercise node behavior. Buildwise framework validation with the full profile also passed.

Docker, `ros2`, and `colcon` were not on the reviewing Windows environment's PATH; PyYAML was absent. No ROS build, ROS test suite, Docker build/start, mock runtime, or hardware operation was performed. The `git submodule status` command encountered a Windows Git shell signal-pipe error; Gitlink observations above use the successful read-only `git ls-tree` command instead.

### 2026-10-21 control-wrapper regression coverage

The control package registers
[test_generate_control_xacro.py](../../ros_ws/src/ur16e_mir250_control/test/test_generate_control_xacro.py)
through `ament_cmake_pytest`. Its 16 cases **passed**: old, required, defaulted
and multiline macro parameters; misleading defaults and unrelated macro names;
missing or duplicate target macros; rendered XML attribute semantics; and
missing or repeated template markers. Tests use local macro-interface fixtures
without downloading dependencies or starting ROS nodes.

From a sourced `ros_ws` directory, run the focused cases directly:

```bash
python3 -m pytest src/ur16e_mir250_control/test/test_generate_control_xacro.py -v
```

The application image rebuilt all 58 packages with the generated wrapper. The
resulting `urmir-jazzy:latest` image ID is
`sha256:60be90bce31c757d3371129cd7d1ce1dc8d4cf17c3124c5e013bf880fd8b9993`.
Verification passed 23 tests: the 16 generator cases and all seven existing
model-consistency cases.

Eight complete Xacro expansions also passed: the exact upstream 3.8.0 control
macro and the installed 3.9.0 macro, each with both `none` and `kenos` and with
`use_mock_hardware` set to both `true` and `false`. The older-macro checks used
current versions of the other dependencies; they do not establish compatibility
with a complete old driver installation at runtime. These checks started no ROS
nodes and operated no hardware. Mock bringup, GUI, and hardware behavior remain
unverified.

### 2026-09-09 model verification

Six independent Xacro scenarios passed using existing WSL Python 3.12.3 and
PyYAML 6.0.1, isolated upstream sources and a temporary package index:

- `none` and `kenos`, each with the stored UR16e calibration.
- Both tools with a modified calibration (shoulder Z +0.0123 m, forearm X -0.0045 m).
- Both tools with `ur_type=ur5e` and the upstream UR5e calibration.

The temporary check extracts Xacro mappings from the current RSP, MoveGroup and
MTC launch-file ASTs and supplies concrete launch-variable values. It performs
real Xacro expansion, but does not resolve ROS launch include scopes or execute
the registered pytest suite. RSP is expanded with control enabled and the
planners with control disabled. Full link/joint definitions agree after excluding
`safety_controller` tags and root-level driver metadata. The no-tool model has
28 links/27 joints; Kenos has 30 links/29 joints. SRDF references and tree checks
passed. The changed calibration affects exactly the shoulder-pan and elbow joint
origins. The previous empty planner mappings fail model comparison for both tools.

Dependency revisions used for these successful checks:

| Dependency | Ref | Commit |
| --- | --- | --- |
| [Xacro](https://github.com/ros/xacro) | `ros2` | `da4b3849f8320903d625250089f67f0632be86f2` |
| [ament index](https://github.com/ament/ament_index) | `jazzy` | `2ccdcb8fd19a364fc793273b5793459207a440b9` |
| [UR description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) | `jazzy` | `39242984dc8d1fff9584c922c17c69c58df3591d` |
| [UR driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/3.2.0) | `3.2.0` | `d497de33da6e756cd6b7c1e307a43d9bc41f60ab` |

During that verification, newer driver Jazzy revision
`404ca549bc0b441821c56d781238e8c3b6103e3e` failed expansion because
`ur_ros2_control` required `ur_type`, which the then-current local wrapper omitted.
The failure also reproduced with the pre-change RSP mapping, independently of
the model-input correction. The 2026-10-21 [wrapper generation](#ur-control-wrapper-generation)
change now supplies the attribute conditionally from the
[control template](../../ros_ws/src/ur16e_mir250_control/urdf/ur16e_mir250.urdf.xacro.in).
The successful older tag above was a verification dependency, not a pinned
deployment dependency or confirmation of the robot system's installed version.

The registered ROS pytest suite, compilation and runtime checks remain unrun.
No global dependencies were installed and no ROS nodes or hardware ran.
Final static checks passed for ten Python files, 25 XML files and 112 relative
documentation links, along with Git whitespace and Buildwise full-profile checks.

### 2026-09-10 executor regression coverage

The MTC package now registers two separate test executables under `BUILD_TESTING`:

- [test_execution_sequence.cpp](../../ros_ws/src/ur16e_mtc_gpio_stage/test/test_execution_sequence.cpp)
  exercises the production ordering helper without ROS. Seven cases cover empty
  sequences, operation-before-effect ordering with empty/IO stages, motion/IO/scene
  failures, exception propagation, and preserving vacuum after a failure. The
  callbacks simulate operations and scene state; this does not test ROS transport.
- [test_scene_effects.cpp](../../ros_ws/src/ur16e_mtc_gpio_stage/test/test_scene_effects.cpp)
  registers four GTests with actual MoveIt scene objects and MTC serialization.
  It covers world-to-attached-to-placed transitions and collision restoration;
  preserving attachments/metadata while clearing planned joint-state fields;
  full-snapshot replacement while retaining measured joints; and the distinction
  between an empty full scene and an empty diff. It uses an embedded one-joint
  robot model, with no external meshes, nodes or controller operations.

The seven portable cases **passed** on WSL Ubuntu with g++ 13.3.0, compiled using
`-std=c++17 -O2 -DNDEBUG -Wall -Wextra -Wpedantic -Werror`. Checks remain active
under `NDEBUG`. Run directly from the repository root on Linux:

```bash
g++ -std=c++17 -O2 -DNDEBUG -Wall -Wextra -Wpedantic -Werror \
  -Iros_ws/src/ur16e_mtc_gpio_stage/include \
  ros_ws/src/ur16e_mtc_gpio_stage/test/test_execution_sequence.cpp \
  -o /tmp/ur16-execution-sequence-tests
/tmp/ur16-execution-sequence-tests
```

Run both registered executables in a built, sourced ROS Jazzy workspace with
the package's declared test dependencies installed:

```bash
cd ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to ur16e_mtc_gpio_stage --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select ur16e_mtc_gpio_stage --event-handlers console_direct+
colcon test-result --verbose
```

The real MoveIt GTests and ROS package build were **not run**: ROS headers,
libraries and gtest are absent locally. Service acknowledgement, response timeout,
controller completion, the deployed global scene and physical pick/place behavior
remain unverified. Direct compilation of the portable helper is not a build of
`CustomExecutor` or `test_task`. No fake ROS headers were used, and no ROS nodes,
GPIO calls, containers or hardware were started. Buildwise full-profile validation
and Python/XML syntax checks passed; no lint target is registered.
Final whitespace checks and 119 relative documentation links also passed.

### 2026-09-10 valid-plan regression coverage

The third P1 adds two more registered test executables:

- [test_plan_gate.cpp](../../ros_ws/src/ur16e_mtc_gpio_stage/test/test_plan_gate.cpp)
  exercises the production admission-state helper. Ten portable cases cover
  state transitions, execution without a plan, invalid cached storage, failed
  initialization/replanning completion, duplicate execution, busy-state admission,
  invalidation and consuming eligibility before operations. Caller scenarios use
  simulated task storage and operations; these are not ROS callback tests.
- [test_plan_validation.cpp](../../ros_ws/src/ur16e_mtc_gpio_stage/test/test_plan_validation.cpp)
  adds five GTests against actual ROS message types and the production scene
  validator. It checks equivalent ordering/metadata, joint tolerance and malformed
  feedback, twenty collision-relevant mutations, and fifteen partial/malformed
  scene cases. Cases include changed object pose, attachments, ACM, fixed
  transforms, padding/scaling and octomap data. It does not start ROS nodes.

The ten portable gate cases **passed** using WSL g++ 13.3.0 with
`-std=c++17 -O2 -DNDEBUG -Wall -Wextra -Wpedantic -Werror`. Run from the repository
root without ROS:

```bash
g++ -std=c++17 -O2 -DNDEBUG -Wall -Wextra -Wpedantic -Werror \
  -Iros_ws/src/ur16e_mtc_gpio_stage/include \
  ros_ws/src/ur16e_mtc_gpio_stage/test/test_plan_gate.cpp \
  -o /tmp/ur16-plan-gate-tests
/tmp/ur16-plan-gate-tests
```

The package-level colcon commands above also run these new tests. The five
ROS-message GTests and full ROS node build remain **unrun** locally because ROS
headers/libraries/gtest are unavailable. Freshness checks, callback scheduling,
service timeouts, preview clearing through Unity and live start-scene comparison
still require ROS integration verification. No ROS nodes, containers or robot
hardware were started for this correction. The exact-geometry comparison is
conservative; representation differences may reject a physically equivalent scene.
Final Python/XML syntax, whitespace, 127 relative documentation links and
Buildwise full-profile checks passed. No lint target is registered.

### 2026-09-14 basic motion frame/TCP regression coverage

The commander package registers two test executables under `BUILD_TESTING`:

- [test_command_sequence.cpp](../../ros_ws/src/ur16e_commander/test/test_command_sequence.cpp)
  exercises the production sequential-move helper. Eight portable cases cover
  planning after preceding completion, false returns and exceptions from both
  phases at every move index, duplicate targets, empty commands and stateful
  callbacks. Earlier completed operations remain intact; no later operations run
  after failure. These are simulated callbacks, not MoveIt/controller operations.
- [test_pose_targets.cpp](../../ros_ws/src/ur16e_commander/test/test_pose_targets.cpp)
  contains nine GTests with actual ROS messages and tf2 geometry. Cases cover
  equivalent targets in translated/rotated frames, one array lookup with the
  requested timestamp, rejection of malformed later poses before lookup, invalid
  frames/times/empty requests, propagated TF errors, quaternion validation,
  malformed transform results, wrist-to-TCP composition and invalid/overflowed
  transforms. The lookup callback is supplied by the test; no ROS nodes run.

The eight portable cases **passed** with WSL Ubuntu-24.04 g++ 13.3.0 using C++17,
`NDEBUG` and warnings as errors. Run from the repository root on Linux:

```bash
g++ -std=c++17 -O2 -DNDEBUG -Wall -Wextra -Wpedantic -Werror \
  -Iros_ws/src/ur16e_commander/include \
  ros_ws/src/ur16e_commander/test/test_command_sequence.cpp \
  -o /tmp/ur16-test-command-sequence
/tmp/ur16-test-command-sequence
```

The nine ROS-message/tf2 GTests and full commander build remain **unrun** locally:
this environment has no ROS installation. In a ROS Jazzy development workspace
with the declared dependencies installed, build and run the registered tests:

```bash
cd ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to ur16e_commander --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select ur16e_commander --event-handlers console_direct+
colcon test-result --verbose
```

The package now directly declares geometry messages, standard messages,
parameter descriptors, MoveIt core and tf2/TF-listener dependencies. No lint
target is registered. Portable test execution does not compile the ROS node or
verify subscriptions, TF transport/history/timeouts, model loading, action
results or Unity integration. Callback/executor and Jazzy API behavior received
source review only. No ROS nodes, containers or hardware were started.

At this stage the shell publishers were labeled legacy wrist examples and
required the startup compatibility opt-in; they were not regression tests. The
standalone `test_moveit` demo also explicitly keeps its wrist endpoint. See
[API migration and execution limits](API.md).

Final static checks passed: ten Python AST parses, 25 XML parses, 154 relative
documentation links, Git whitespace checks and Buildwise full-profile validation.

### 2026-09-14 mounted-tool capability regression coverage

The MTC package registers three more test executables:

- [test_tool_definition.cpp](../../ros_ws/src/ur16e_mtc_gpio_stage/test/test_tool_definition.cpp)
  has six portable cases covering the two startup definitions, unknown names,
  rejecting a missing capability before simulated dependent work, mismatched
  capabilities, twelve malformed contact/IO bindings and equivalent contact-link
  ordering. These tests use the production resolver/requirement helper; the
  simulated callbacks do not test ROS subscriptions or service transport.
- [test_tool_model_validation.cpp](../../ros_ws/src/ur16e_mtc_gpio_stage/test/test_tool_model_validation.cpp)
  has eight GTests using actual MoveIt models built from embedded URDF/SRDF with
  box collision geometry. They cover matching none/Kenos models, the real SRDF
  chain's base-link collision inclusion, selected-tool/model mismatches, missing
  links/groups, movable or wrong topology, missing/extra collision geometry,
  incorrect IK frames and allowed Kenos calibration offsets. No meshes or robot
  nodes are needed.
- [test_startup_parameter.cpp](../../ros_ws/src/ur16e_mtc_gpio_stage/test/test_startup_parameter.cpp)
  has six actual-rclcpp tests for defaults, preserved auto-declared overrides,
  overrides without automatic declaration, existing read-only parameters and
  rejection of wrong types or unsupported predeclared mutable parameters. The
  tests create local node objects with parameter services/events disabled and do
  not spin an executor, launch controllers or operate hardware.

The six portable capability cases **passed** with existing WSL g++ 13.3.0,
C++17, `NDEBUG` and warnings as errors. Run from the repository root on Linux:

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror -DNDEBUG \
  -Iros_ws/src/ur16e_mtc_gpio_stage/include \
  ros_ws/src/ur16e_mtc_gpio_stage/test/test_tool_definition.cpp \
  -o /tmp/ur16-test-tool-definition
/tmp/ur16-test-tool-definition
```

The fourteen new ROS-dependent tests and full MTC node build are **unrun**
locally because ROS is unavailable. The existing MTC package `colcon build` and
`colcon test` commands above include all three new executables. The package now
declares `rcl_interfaces` directly for startup parameter descriptors. No lint
target is registered. Runtime capability rejection, retained preview/status
delivery, installed model loading and service/action behavior require ROS
integration verification. No ROS nodes, containers or hardware were started.

Final static checks passed: ten Python AST parses, 25 XML parses, 167 relative
documentation links, Git whitespace checks and Buildwise full-profile validation.

### 2026-10-21 TCP shell examples

The single-pose and array shell publishers now send explicit TCP goals on
`/tcp_pose_target` and `/tcp_pose_array_target`, using timestamp zero for latest
TF. The sample numbers describe the TCP rather than preserving the previous
wrist configurations. They require no legacy topic opt-in. See the
[README testing instructions](../../README.md#testing).

Bash syntax checks and execution with a print-only `ros2` stub passed for both
scripts. The captured payloads passed checks for topic/type, one-shot publication,
reference frame, zero timestamp, pose count, finite coordinates and unit
quaternions. Git whitespace checks passed. No ROS nodes, controllers, containers
or hardware were started; planning/reachability and actual execution remain
unverified. No application build or full ROS test suite was run for this
shell/documentation change; no lint target is registered.

### 2026-10-22 reusable Safe&Light model

The `kenos_flexigrip` package registers [test_kenos_sl.py](../../ros_ws/src/kenos-flexigrip/test/test_kenos_sl.py)
through `ament_cmake_pytest` when `BUILD_TESTING` is enabled; its manifest declares
Xacro as a test dependency. The suite can also run directly with Python and Xacro
on the import path:

```bash
python3 ros_ws/src/kenos-flexigrip/test/test_kenos_sl.py -v
```

Six cases passed with Python 3.12.14, Xacro 2.1.1 and PyYAML 6.0.3. Dependencies
were loaded from an isolated temporary folder without modifying the installed
Python environment. Each case expands an actual wrapper around
[kenos_sl.urdf.xacro](../../ros_ws/src/kenos-flexigrip/urdf/kenos_sl.urdf.xacro).
Coverage includes the fixed flange chain and nominal TCP pose, mounting yaw
preserving eccentricity, mounting/TCP calibration overrides, two prefixed
instances and collision containment of CAD envelope checkpoints, including
the ejector and air fitting.

The standalone run does not execute ament/CMake test registration or a ROS package
build. There is no registered lint target in this package. RViz, MoveIt planning,
physical mounting, payload and vacuum IO were not exercised. The macro is now
registered as `tool:=kenos_sl`; see [the tool model](ROBOT_AND_TOOLS.md#safelight-geometry-and-calibration).

### 2026-10-22 Safe&Light toolchain integration

The launch/model suite now has eleven cases. In addition to the existing checks,
it covers all three selected RSP wrappers, all five Safe&Light calibration values
at every local include boundary, the nominal eccentric TCP, combined mounting/TCP
and arm-calibration overrides, and isolation from FlexiGrip TCP inputs. Actual
launch substitutions, Xacro commands and MoveIt builders are used; the upstream
driver lifecycle is outside this suite. These eleven ROS-dependent cases remain
unrun locally.

The portable tool-definition suite now has seven cases covering all three startup
definitions, shared vacuum IO, capability admission, malformed/cross-tool bindings
and order-independent contact lists. All seven passed with the existing WSL g++,
C++17, `NDEBUG` and warnings as errors. The same compile/run command above applies.
The MoveIt model suite now has ten GTests, adding all selected/model combinations,
foreign-tool contamination and both vacuum tools' topology/geometry/calibration
checks. These ten tests and the full ROS package build remain unrun without ROS.

Four direct model scenarios passed: `none`, `kenos`, `kenos_sl`, and Safe&Light
with all five nondefault mounting/TCP values. Each expanded with `use_control`
both false and true (eight URDFs) plus its SRDF (four). The control/no-control
variants have identical link/joint definitions; each model has a unique rooted
tree and all SRDF link/joint references resolve. No-tool has 28 links/27 joints;
each vacuum tool has 30/29. The default Safe&Light flange TCP is
`(0.150, 0, -0.034)` m with outward +Z. The override scenario used height `0.082`,
radius `0.057`, yaw `pi/2`, TCP XYZ `0.015 -0.010 0.095` and RPY `0.1 0.2 -0.3`;
its resulting flange position `(0.177, 0.015, -0.010)`, calibrated orientation
and cylinder dimensions passed checks.

These expansions used existing WSL Python 3.12.3/PyYAML 6.0.1 and isolated official
source checkouts: Xacro `da4b3849f8320903d625250089f67f0632be86f2`,
ament_index `2ccdcb8fd19a364fc793273b5793459207a440b9`, UR description
`39242984dc8d1fff9584c922c17c69c58df3591d`, and UR driver tag `3.2.0`
(`d497de33da6e756cd6b7c1e307a43d9bc41f60ab`). Actual package-index entries point
at these sources and the local packages; no ROS/launch APIs were stubbed. Direct
Xacro mappings do not establish ROS launch lifecycle behavior or compatibility
with a different installed driver version; the earlier upstream compatibility
issue is handled by the [control-wrapper generation](#ur-control-wrapper-generation) merged from WSL; its commissioning limits remain in [known issues](../ai-context/KNOWN_ISSUES.md).

Additional checks passed: the six standalone Safe&Light Xacro cases, twelve Python
syntax parses, 26 XML/Xacro parses, Bash syntax and help for both edited scripts,
shell selection/argument forwarding for all three tools and multiword export
arguments, relative documentation links, Git whitespace and framework full-profile
validation. Shell forwarding checks used temporary command recorders, not ROS
processes. No lint target is registered. No nodes, containers, planning/execution,
vacuum calls or hardware operations were run.

### 2026-10-22 combined-history verification

After combining the Windows Safe&Light commits with the WSL compatibility fix,
the actual generator produced a temporary installed-style control package from
the pinned 3.2.0 driver signature. Its package-index entry points at the generated
wrapper rather than the raw template directory. All four model scenarios above
passed again: eight control/no-control URDFs and four SRDFs, with matching physical
geometry, valid trees/references, selected groups and nominal/overridden TCPs.
Fourteen Python and 26 XML/template parses, 193 relative documentation links,
merge-marker checks and framework validation passed. The vendored patch's required
context whitespace is excluded from Git whitespace checking; its diff syntax
parses successfully. The sixteen generator pytest cases were not rerun because
pytest is absent in the host WSL environment. No Docker, ROS-node, workspace-build
or hardware operation was performed for this merge verification.

### 2026-10-22 exact TCP A/B plan and review

The applied exact TCP package compiled completely, including `test_task`, with GCC 13.3.0,
ROS Jazzy and installed MTC 0.1.4 in existing `urmir-jazzy:latest` image
`sha256:752aa09527488fce5baaa35ebbff1aad52c6e82e72a302db4f14ded79f2f615b`.
Validation used an isolated, network-disabled, read-only container with writable
temporary build outputs and `ROS_LOG_DIR=/tmp/ros-logs`; it started no application
node, controller, launch, robot connection or hardware operation. Parameter tests
construct in-process nodes without an executor or parameter services.

All six new TCP request/correlation/admission cases pass, as do all eight startup
parameter cases, including the two immutable-bool cases. Plan-validation,
tool-model, PlanGate, executor-ordering and tool-definition suites pass. Seven
of eight CTest targets pass. The unchanged
`SceneEffectsTest.SerializedPickPlaceEffectsUpdateTheLiveSceneInOrder` fails at
`test_scene_effects.cpp:166`: MoveIt removes the tile/hand ACM entry while the
test expects an explicit entry. Compiling the untouched baseline against the
same image reproduces the identical failure; the other three scene-effect cases
pass. This proposal does not change the scene-effect implementation or fixture.

The exact TCP checks validate complete two-pose/frame/token input, quaternion
normalization, preservation of full poses, and matching mode/ID before one-shot
gate consumption. They do not prove planning reachability, ROS-TCP delivery,
controller behavior, physical alignment or hardware execution. The legacy basic
TCP topics continue to plan and execute immediately; the new plan-only interface
has separate topics and requires an explicit reviewed-ID execution request.

### 2026-10-22 tool-contact targets and execution-frame validation

The changed MTC package, including the application executable, compiled in the
existing local ROS Jazzy/MTC 0.1.4 image with GCC 13.3.0. The dedicated validation
container used `--network none` and a read-only source mount; writable build
outputs were isolated from the repository. No controller, real-stack launch,
robot connection, hardware motion or IO was run.

All nine plan-validation cases pass, including four new regressions for
MoveIt's repeated target frame with distinct source keys, unused controller-TCP
encoder variation, and preservation of referenced frame/geometry checks. All
four new contact-target cases pass: rotated object-centre offsets, preservation
of contact/normal under twist, validation of both poses before scene mutation,
and quaternion normalization without moving the contact point. Existing TCP,
startup-parameter, model, gate, execution-order and tool-definition suites pass.
Eight of nine CTest targets pass. The sole failure remains the unchanged
`SceneEffectsTest.SerializedPickPlaceEffectsUpdateTheLiveSceneInOrder` assertion
at `test_scene_effects.cpp:166`, already reproduced on the previous baseline
against this image. These changes do not alter its scene-effect implementation
or test fixture.

A subsequent isolated integration smoke started the new application executable
with the captured live URDF/SRDF and joint snapshot, installed KDL/OMPL and joint
limits, in-memory Get/ApplyPlanningScene services and static joint feedback.
The separately advertised contact endpoint produced a full MTC solution and a
74-point preview, including the new independent place-twist alternatives.
Switching to exact TCP A/B then removed only the same node's unchanged,
unexecuted preview object and produced an 11-point preview; the ground object
remained. Both requests were plan-only inside `--network none`, with no Execute
request, driver or real ROS stack. Logs and JSON proof are stored in the Unity
project's `Tools/RobotFrames/BackendWorkflowBuild/` directory.

These checks do not establish ROS-TCP delivery, physical compression, payload,
collision-model fidelity or hardware execution. The Nordbo deployment is
unchanged. No lint target is registered in the package.

## Refresh

- Last refreshed: 2026-10-22 for exact TCP A/B planning, Safe&Light integration and the merged WSL compatibility fixes.
- Source basis: tool macros, composition/SRDF and launch files, capability/model tests, control-wrapper template/generator and tests, Dockerfiles and MTC dependency-order patch, package/build configuration, and the dated verification records above.
- Limitations: the October 21 WSL builds and 23 tests cover the pre-Safe&Light tree; October 22 direct model expansions and portable checks cover the new tool. Full combined workspace build, the expanded eleven-case launch suite and ten MoveIt model cases remain unrun. Older-macro checks use current other dependencies. GUI, mock bringup, physical calibration, hardware behavior and Nordbo deployment remain unverified.
