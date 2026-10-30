#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/ros_ws/install/setup.bash
python3 /tmp/replay-source/serialize_captured_scenes.py /tmp/snapshot.json /tmp/result
cmake -S /tmp/replay-source -B /tmp/replay-build
cmake --build /tmp/replay-build -j 2
set +e
/tmp/replay-build/validate_captured_scenes /tmp/result/planned.cdr /tmp/result/current.cdr
patched_code=$?
set -e
cmake -S /tmp/replay-source -B /tmp/baseline-replay-build \
  -DVALIDATION_INCLUDE_ROOT=/tmp/baseline-source/include
cmake --build /tmp/baseline-replay-build -j 2
set +e
/tmp/baseline-replay-build/validate_captured_scenes /tmp/result/planned.cdr /tmp/result/current.cdr
baseline_code=$?
printf '{"patched_validation_exit":%s,"baseline_validation_exit":%s,"snapshot":"mtc_execute_scene_live_20261028.json","ros_nodes_started":false,"network":"none"}\n' \
  "$patched_code" "$baseline_code" > /tmp/result/captured-scene-result.json
if [ "$patched_code" -ne 0 ] || [ "$baseline_code" -ne 1 ]; then exit 1; fi
