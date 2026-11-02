"""Activate the approved tested image after a fresh passive idle check, with rollback."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

STAGE = Path('/home/nordbo/.cache/codex/fixture-tiling-20261102')
CONTAINER = 'urmir-jazzy'
NO_COMMANDS = {'robot_motion_requested': False, 'execution_requests_sent': False,
               'io_commands_sent': False, 'production_plan_requests_sent': False}


def run(args, **kwargs):
    return subprocess.run(args, check=True, capture_output=True, **kwargs)


def inspect():
    return json.loads(run(['docker', 'inspect', CONTAINER], text=True).stdout)[0]


def probe():
    result = run(['docker', 'exec', '-i', CONTAINER, 'bash', '-c',
                  'source /opt/ros/jazzy/setup.bash && source /home/ros_ws/install/setup.bash && python3 -'],
                 input=(STAGE / 'probe_ros.py').read_text(), text=True, timeout=35)
    return json.loads(result.stdout)


def resume():
    # Keep the exact previous launch arguments/environment private on Nordbo.
    run(['docker', 'cp', str(STAGE / 'backup/runtime.json'), CONTAINER + ':/tmp/codex-fixture-runtime.json'])
    script = r'''import json,subprocess,os
from pathlib import Path
p=Path('/tmp/codex-fixture-runtime.json');p.chmod(0o600)
launch=json.loads(p.read_text())['launch'];log=Path('/tmp/codex-fixture-bringup.log').open('a')
job=subprocess.Popen(launch['args'],cwd=launch['cwd'],env=launch['environment'],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,close_fds=True)
print(json.dumps({'pid':job.pid,'log':'/tmp/codex-fixture-bringup.log','args':launch['args']}))'''
    return json.loads(run(['docker', 'exec', CONTAINER, 'python3', '-c', script], text=True).stdout)


def compose(root, env):
    return run(['docker', 'compose', '--project-name', 'docker', '--file',
                str(Path(root) / 'docker/docker-compose.yaml'), 'up', '-d', '--no-build',
                '--pull', 'never', 'ur_container'], text=True, env=env, timeout=75)


def main():
    staged = json.loads((STAGE / 'staging.json').read_text())
    built = json.loads((STAGE / 'build-result.json').read_text())
    tests = json.loads((STAGE / 'test-result.json').read_text())
    source = json.loads((STAGE / 'source-sync.json').read_text())
    assert built['exitCode'] == 0 and tests['success'] and source['success']
    assert len(tests['replays']) == 2 and all(r['exitCode'] == 0 and r['result'].get('passed') for r in tests['replays'])
    assert inspect()['Id'] == staged['oldContainerId']
    old = json.loads((STAGE / 'backup/container-inspect.json').read_text())[0]
    environment = dict(os.environ)
    old_env = dict(v.split('=', 1) for v in old['Config']['Env'] if '=' in v)
    environment['DISPLAY'] = old_env.get('DISPLAY', '')
    root = source['activeSourceCheckout']
    run(['docker', 'compose', '--project-name', 'docker', '--file', root + '/docker/docker-compose.yaml',
         'config', '--quiet'], text=True, env=environment)
    before = probe()
    (STAGE / 'preflight.json').write_text(json.dumps(before, indent=2))
    assert before['idle'], 'Robot/action state is not stationary and idle; production was not changed'
    alias = json.loads(run(['docker', 'image', 'inspect', 'urmir-jazzy:latest'], text=True).stdout)[0]
    assert alias['Id'] == staged['oldImage'], 'Production image alias changed during validation'
    runtime = json.loads((STAGE / 'backup/runtime.json').read_text())
    launch = runtime['launch']
    check_launch = r'''import os,json,signal,sys
from pathlib import Path
expected=json.load(sys.stdin);pid=expected['pid']
actual=Path('/proc/'+str(pid)+'/cmdline').read_bytes().decode().strip('\0').split('\0')
assert actual==expected['args'],'Launch process changed'
os.kill(pid,signal.SIGINT)'''
    changed = False
    try:
        run(['docker', 'exec', '-i', CONTAINER, 'python3', '-c', check_launch], input=json.dumps(launch), text=True)
        changed = True
        for _ in range(30):
            exists = subprocess.run(['docker', 'exec', CONTAINER, 'test', '-d', '/proc/' + str(launch['pid'])], capture_output=True).returncode == 0
            if not exists: break
            time.sleep(.5)
        else:
            raise RuntimeError('The original ROS launch did not stop gracefully')
        run(['docker', 'tag', built['image'], 'urmir-jazzy:latest'])
        compose(root, environment)
        active = inspect()
        assert active['Image'] == built['image'] and active['State']['Running']
        assert any(m.get('Destination') == '/root/.config/ur16e' and m.get('Type') == 'volume' for m in active['Mounts'])
        launched = resume()
        final = None
        for attempt in range(6):
            time.sleep(3)
            try:
                final = probe()
                models = [final['models'][n] for n in ('/robot_state_publisher', '/move_group', '/mtc_test_task_node')]
                if all(isinstance(m.get('mount_height_m'), (int, float)) and abs(m['mount_height_m']-.05525) < 1e-8 for m in models) and final['idle'] and \
                   '/save_fixture_calibration' in final['services'] and '/fixture_tiling_plan_target' in final['topics']:
                    break
            except (subprocess.SubprocessError, ValueError):
                pass
        else:
            raise RuntimeError('Updated ROS graph/model/idle checks did not become healthy')
        signatures = [m.get('robot_link_joint_sha256') for m in models]
        assert all(signatures) and len(set(signatures)) == 1, 'Robot model consumers disagree'
        for name in ('/robot_state_publisher', '/move_group', '/mtc_test_task_node'):
            assert final['models'][name].get('safety_controller_metadata') == before['models'][name].get('safety_controller_metadata'), 'Existing safety configuration changed: ' + name
        assert final['models']['/mtc_test_task_node']['fixture_enabled'] is False, 'Unexpected or missing fixture calibration state'
        final.update({'success': True, 'commit': source['commit'], 'image': built['image'],
                      'containerId': active['Id'], 'launch': launched,
                      'calibrationVolume': [m for m in active['Mounts'] if m.get('Destination') == '/root/.config/ur16e'],
                      **NO_COMMANDS})
        (STAGE / 'final-verification.json').write_text(json.dumps(final, indent=2) + '\n')
        result = {'success': True, 'rolledBack': False, 'image': built['image'],
                  'sourceCheckout': root, 'launch': launched, 'allModelConsumersRestarted': True, **NO_COMMANDS}
    except Exception as error:
        result = {'success': False, 'error': str(error), 'rolledBack': False, **NO_COMMANDS}
        if changed:
            run(['docker', 'tag', staged['oldImage'], 'urmir-jazzy:latest'])
            compose(staged['hostRoot'], environment)
            result['rollbackLaunch'] = resume()
            result['rolledBack'] = True
        (STAGE / 'activation-result.json').write_text(json.dumps(result, indent=2) + '\n')
        raise
    (STAGE / 'activation-result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
