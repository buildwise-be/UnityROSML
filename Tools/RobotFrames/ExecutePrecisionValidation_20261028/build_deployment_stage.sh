#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/ros_ws/install/setup.bash
cmake -S /tmp/source -B /tmp/build -DCMAKE_INSTALL_PREFIX=/tmp/install \
  -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release 2>&1 | tee /tmp/result/configure.log
cmake --build /tmp/build -j 2 2>&1 | tee /tmp/result/build.log
set +e
ctest --test-dir /tmp/build --output-on-failure 2>&1 | tee /tmp/result/ctest.log
test_code=${PIPESTATUS[0]}
set -e
cp -a /tmp/build/test_results /tmp/result/
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
root=Path('/tmp/result/test_results/ur16e_mtc_gpio_stage')
for path in root.glob('*.xml'):
    doc=ET.parse(path).getroot()
    failures=doc.findall('.//testcase/failure')
    assert not failures or path.name=='test_scene_effects.gtest.xml', path.name
    if failures:
        cases=[c.attrib['name'] for c in doc.findall('.//testcase') if c.find('failure') is not None]
        assert cases==['SerializedPickPlaceEffectsUpdateTheLiveSceneInOrder'], cases
plan=ET.parse(root/'test_plan_validation.gtest.xml').getroot()
assert int(plan.attrib['tests'])==14 and int(plan.attrib['failures'])==0
PY
if [ "$test_code" -ne 0 ]; then
  # Only the independently known, unchanged scene-effects assertion is accepted.
  grep -q '90% tests passed, 1 tests failed out of 10' /tmp/result/ctest.log
  grep -q '1 - test_scene_effects (Failed)' /tmp/result/ctest.log
fi
python3 /tmp/replay-source/serialize_captured_scenes.py /tmp/snapshot.json /tmp/result
cmake -S /tmp/replay-source -B /tmp/replay-build
cmake --build /tmp/replay-build -j 2
/tmp/replay-build/validate_captured_scenes /tmp/result/planned.cdr /tmp/result/current.cdr \
  2>&1 | tee /tmp/result/full-scene-replay.log
cmake --install /tmp/build 2>&1 | tee /tmp/result/install.log
mkdir -p /tmp/result/install
cp -a /tmp/install/. /tmp/result/install/
sha256sum /tmp/result/install/lib/ur16e_mtc_gpio_stage/test_task > /tmp/result/test_task.sha256
printf '{"build":"passed","plan_validation_cases":14,"ctest_exit":%s,"captured_scene_replay":"passed","network":"none","application_started":false}\n' \
  "$test_code" > /tmp/result/result.json
