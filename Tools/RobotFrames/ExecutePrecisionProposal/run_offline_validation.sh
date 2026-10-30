#!/usr/bin/env bash
# Explicitly run only inside the approved local --network none validation container.
set -o pipefail
source /opt/ros/jazzy/setup.bash
source /home/ros_ws/install/setup.bash
set -u

cmake -S /tmp/source -B /tmp/patched-build -DCMAKE_INSTALL_PREFIX=/tmp/patched-install \
  -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release 2>&1 | tee /tmp/result/configure.log
configure_code=${PIPESTATUS[0]}
if [ "$configure_code" -ne 0 ]; then exit "$configure_code"; fi
cmake --build /tmp/patched-build -j 2 2>&1 | tee /tmp/result/build.log
build_code=${PIPESTATUS[0]}
if [ "$build_code" -ne 0 ]; then exit "$build_code"; fi
ctest --test-dir /tmp/patched-build --output-on-failure 2>&1 | tee /tmp/result/ctest.log
ctest_code=${PIPESTATUS[0]}
cp -r /tmp/patched-build/test_results /tmp/result/

# Compile the untouched HEAD header with the new regression tests, and the
# unchanged scene-effect test, to distinguish the correction from a baseline.
cmake -S /tmp/baseline-source -B /tmp/baseline-build -DCMAKE_INSTALL_PREFIX=/tmp/baseline-install \
  -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release 2>&1 | tee /tmp/result/baseline-configure.log
baseline_configure_code=${PIPESTATUS[0]}
if [ "$baseline_configure_code" -ne 0 ]; then exit "$baseline_configure_code"; fi
cmake --build /tmp/baseline-build --target test_plan_validation test_scene_effects -j 2 \
  2>&1 | tee /tmp/result/baseline-build.log
baseline_build_code=${PIPESTATUS[0]}
if [ "$baseline_build_code" -ne 0 ]; then exit "$baseline_build_code"; fi
/tmp/baseline-build/test_plan_validation \
  --gtest_filter=PlanValidation.RecordedMoveItQuaternionRoundoffDoesNotRejectExecution \
  --gtest_output=xml:/tmp/result/baseline-recorded.gtest.xml 2>&1 | tee /tmp/result/baseline-recorded.log
baseline_recorded_code=${PIPESTATUS[0]}
/tmp/baseline-build/test_scene_effects \
  --gtest_filter=SceneEffectsTest.SerializedPickPlaceEffectsUpdateTheLiveSceneInOrder \
  --gtest_output=xml:/tmp/result/baseline-scene-effects.gtest.xml 2>&1 | tee /tmp/result/baseline-scene-effects.log
baseline_scene_effects_code=${PIPESTATUS[0]}
sha256sum /tmp/patched-build/test_task > /tmp/result/test_task.sha256
printf '{"configure_exit":%s,"build_exit":%s,"ctest_exit":%s,"baseline_configure_exit":%s,"baseline_build_exit":%s,"baseline_recorded_exit":%s,"baseline_scene_effects_exit":%s,"hardware":false,"deployment":false,"container_network":"none","application_started":false}\n' \
  "$configure_code" "$build_code" "$ctest_code" "$baseline_configure_code" "$baseline_build_code" \
  "$baseline_recorded_code" "$baseline_scene_effects_code" > /tmp/result/result.json

# CTest and baseline failure details are reviewed from their logs/XML; they are
# retained instead of being disguised by changing or excluding their tests.
exit 0
