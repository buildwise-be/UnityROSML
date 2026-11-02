"""Run the freshly built application tests and fixture replays without ROS network access."""
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

STAGE = Path('/home/nordbo/.cache/codex/fixture-tiling-20261102')
BUILD = '/home/ros_ws/build_codex_fixture_20261102'
PACKAGES = 'kenos_flexigrip mir_description ur16e_bringup ur16e_commander ur16e_environment ur16e_interfaces ur16e_mir250_control ur16e_mir250_description ur16e_mir250_moveit_config ur16e_mtc_gpio_stage'
KNOWN_BASELINE = 'ur16e_mtc_gpio_stage.SceneEffectsTest.SerializedPickPlaceEffectsUpdateTheLiveSceneInOrder'
REQUIRED_REPORTS = {
    'ur16e_bringup/model_consistency.xunit.xml': 11,
    'ur16e_mtc_gpio_stage/test_fixture_tiling_geometry.gtest.xml': 12,
    'ur16e_mtc_gpio_stage/test_fixture_cycle_builder.gtest.xml': 5,
    'ur16e_mtc_gpio_stage/test_startup_robot_model.gtest.xml': 1,
    'ur16e_mtc_gpio_stage/test_tool_model_validation.gtest.xml': 10,
    'ur16e_mtc_gpio_stage/test_plan_validation.gtest.xml': 14,
}


def assess_results(reports, failures, replays, unreadable_reports=(), ctest_failed_targets=()):
    """Accept complete passing coverage plus only the independently known assertion."""
    incomplete = []
    for relative, minimum_cases in REQUIRED_REPORTS.items():
        matches = [report for report in reports if report['path'].endswith('/test_results/' + relative)]
        if len(matches) != 1 or matches[0]['tests'] < minimum_cases or matches[0]['failed'] or matches[0].get('skipped', 0):
            incomplete.append(relative)
    def known_assertion(failure):
        message = failure['message']
        return failure['name'] == KNOWN_BASELINE and 'test_scene_effects.cpp:166' in message and \
            'getEntry("tile", "hand", permission)' in message and 'Actual: false' in message and 'Expected: true' in message
    unexpected = [failure for failure in failures if not known_assertion(failure)]
    if sum(known_assertion(failure) for failure in failures) > 1:
        unexpected.append({'name': KNOWN_BASELINE, 'message': 'The known assertion appeared more than once'})
    unexpected_targets = [target for target in ctest_failed_targets
                          if target != {'package': 'ur16e_mtc_gpio_stage', 'target': 'test_scene_effects'}]
    expected_replays = {'full-cycle-replay': False, 'full-cycle-home-replay': True}
    replays_complete = len(replays) == 2 and {item['name'] for item in replays} == set(expected_replays) and all(
        item['exitCode'] == 0 and item['result'].get('passed') is True and
        item['result'].get('request_return_home_each_tile') is expected_replays[item['name']]
        for item in replays)
    return {'success': not incomplete and not unexpected and not unreadable_reports and not unexpected_targets and replays_complete,
            'missingOrIncompleteReports': incomplete, 'unexpectedFailures': unexpected,
            'unreadableReports': list(unreadable_reports), 'unexpectedCtestTargets': unexpected_targets,
            'replaysComplete': replays_complete}


def inner():
    out = Path('/evidence')
    setup = 'source /opt/ros/jazzy/setup.bash && source /home/ros_ws/install/setup.bash && '
    def command(name, text):
        with (out / (name + '.log')).open('w') as log:
            return subprocess.run(['bash', '-c', setup + text], stdout=log, stderr=subprocess.STDOUT).returncode
    test_code = command('colcon-test', 'colcon test --build-base ' + BUILD + ' --install-base /home/ros_ws/install --packages-select ' + PACKAGES + ' --event-handlers console_direct+')
    result_code = command('colcon-results', 'colcon test-result --test-result-base ' + BUILD + ' --verbose')
    reports = []
    failures = []
    unreadable = []
    for path in Path(BUILD).rglob('*.xml'):
        if '/test_results/' not in str(path): continue
        try:
            tree = ET.parse(path).getroot()
        except ET.ParseError as error:
            unreadable.append({'path': str(path), 'error': str(error)})
            continue
        cases = list(tree.iter('testcase'))
        reports.append({'path': str(path), 'tests': len(cases),
                        'failed': sum(c.find('failure') is not None or c.find('error') is not None for c in cases),
                        'skipped': sum(c.find('skipped') is not None for c in cases)})
        for case in cases:
            for kind in ('failure', 'error'):
                failure = case.find(kind)
                if failure is not None:
                    failures.append({'name': case.get('classname', '') + '.' + case.get('name', ''),
                                     'message': failure.get('message', '') + (failure.text or '')})
    replays = []
    replay = '/home/ros_ws/src/ur16e_mtc_gpio_stage/test/fixture_planning_replay.py'
    executable = '/home/ros_ws/install/ur16e_mtc_gpio_stage/lib/ur16e_mtc_gpio_stage/test_task'
    for home in (False, True):
        name = 'full-cycle-home-replay' if home else 'full-cycle-replay'
        code = command(name, 'export ROS_LOG_DIR=/tmp/ros-logs PYTHONDONTWRITEBYTECODE=1; '
                       'UR16_FIXTURE_REPLAY_ISOLATED=1 python3 ' + replay + ' --executable ' + executable +
                       ' --output /evidence/' + name + '.json --timeout 90' + (' --return-home' if home else ''))
        data_path = out / (name + '.json')
        data = json.loads(data_path.read_text()) if data_path.exists() else {}
        replays.append({'name': name, 'exitCode': code, 'result': data})
    ctest_failed_targets = []
    for package in PACKAGES.split():
        failed = Path(BUILD) / package / 'Testing/Temporary/LastTestsFailed.log'
        if failed.is_file():
            ctest_failed_targets.extend({'package': package, 'target': line.split(':', 1)[1]}
                                        for line in failed.read_text().splitlines() if ':' in line)
    assessment = assess_results(reports, failures, replays, unreadable, ctest_failed_targets)
    summary = {**assessment, 'network': 'none', 'colconTestExitCode': test_code,
               'colconResultExitCode': result_code, 'reports': reports, 'failures': failures,
               'knownBaselineFailure': KNOWN_BASELINE, 'requiredReports': REQUIRED_REPORTS,
               'ctestFailedTargets': ctest_failed_targets, 'replays': replays,
               'production_ros_access': False, 'robot_motion_requested': False, 'io_commands_sent': False}
    (out / 'test-result.json').write_text(json.dumps(summary, indent=2) + '\n')


def outer():
    built = json.loads((STAGE / 'build-result.json').read_text())
    assert built['exitCode'] == 0
    output = STAGE / 'test-output'
    output.mkdir(exist_ok=True)
    args = ['docker', 'run', '--rm', '--network', 'none',
            '-v', str(Path(__file__).resolve()) + ':/verify.py:ro',
            '-v', str(output) + ':/evidence', built['image'], 'python3', '/verify.py', 'inner']
    with (STAGE / 'test-driver.log').open('w') as log:
        result = subprocess.run(args, stdout=log, stderr=subprocess.STDOUT)
    if result.returncode != 0:
        raise RuntimeError('Isolated test container failed: ' + str(result.returncode))
    data = json.loads((output / 'test-result.json').read_text())
    (STAGE / 'test-result.json').write_text(json.dumps(data, indent=2) + '\n')


if __name__ == '__main__':
    import sys
    inner() if sys.argv[1:] == ['inner'] else outer()
