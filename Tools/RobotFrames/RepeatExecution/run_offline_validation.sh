#!/usr/bin/env bash
# Run only in the isolated local validation container with --network none.
set -e -o pipefail
source /opt/ros/jazzy/setup.bash
source /home/ros_ws/install/setup.bash
cmake -S /tmp/source -B /tmp/repeat-build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DCMAKE_INSTALL_PREFIX=/tmp/repeat-install 2>&1 | tee /tmp/results/configure.log
cmake --build /tmp/repeat-build -j 2 2>&1 | tee /tmp/results/build.log
set +e
ctest --test-dir /tmp/repeat-build --output-on-failure 2>&1 | tee /tmp/results/ctest.log
test_code=${PIPESTATUS[0]}
set -e
cp -r /tmp/repeat-build/test_results /tmp/results/
sha256sum /tmp/repeat-build/test_task > /tmp/results/test_task.sha256
printf '{"buildExitCode":0,"ctestExitCode":%s,"containerNetwork":"none","hardware":false,"applicationStarted":false,"deployment":false}\n' "$test_code" > /tmp/results/result.json
# Preserve real CTest outcomes, including any pre-existing scene-effect failure.
exit "$test_code"
