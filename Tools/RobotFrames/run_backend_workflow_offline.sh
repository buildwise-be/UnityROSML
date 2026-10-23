#!/usr/bin/env bash
set -o pipefail
source /opt/ros/jazzy/setup.bash
source /home/ros_ws/install/setup.bash
set -u
cmake -S /tmp/source -B /tmp/backend-build -DCMAKE_INSTALL_PREFIX=/tmp/backend-install -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release 2>&1 | tee /tmp/result/configure.log
configure_code=${PIPESTATUS[0]}
if [ "$configure_code" -ne 0 ]; then exit "$configure_code"; fi
cmake --build /tmp/backend-build -j 2 2>&1 | tee /tmp/result/build.log
build_code=${PIPESTATUS[0]}
if [ "$build_code" -ne 0 ]; then exit "$build_code"; fi
ctest --test-dir /tmp/backend-build --output-on-failure 2>&1 | tee /tmp/result/ctest.log
test_code=${PIPESTATUS[0]}
cp -r /tmp/backend-build/test_results /tmp/result/ 2>/dev/null || true
printf '{"configure_exit":%s,"build_exit":%s,"test_exit":%s,"hardware":false,"container_network":"none"}\n' "$configure_code" "$build_code" "$test_code" > /tmp/result/result.json
exit "$test_code"
