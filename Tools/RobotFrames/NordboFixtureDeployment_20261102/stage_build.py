"""Stage the owner-authorized ROS update and build an isolated immutable image.

Run on Nordbo. No live process is signalled and no robot endpoint is called.
Private launch environment and container metadata stay in the remote backup.
"""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
from datetime import datetime, timezone

STAGE = Path('/home/nordbo/.cache/codex/fixture-tiling-20261102')
ROOT = Path('/home/nordbo/Unity/UR16UnityControlV2/UR16UnityControl')
CONTAINER = 'urmir-jazzy'
PACKAGES = 'kenos_flexigrip mir_description ur16e_bringup ur16e_commander ur16e_environment ur16e_interfaces ur16e_mir250_control ur16e_mir250_description ur16e_mir250_moveit_config ur16e_mtc_gpio_stage'.split()


def run(args, **kwargs):
    return subprocess.run(args, check=True, capture_output=True, **kwargs)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(name, value):
    (STAGE / name).write_text(json.dumps(value, indent=2) + '\n')


def build():
    staged = json.loads((STAGE / 'staging.json').read_text())
    base = json.loads(run(['docker', 'image', 'inspect', staged['oldTag']], text=True).stdout)[0]
    assert base['Id'] == staged['oldImage'], 'Rollback tag no longer identifies the exact production image'
    args = ['docker', 'build', '--network', 'none', '--pull=false', '--progress=plain',
            '-t', staged['candidateTag'], str(STAGE / 'source')]
    with (STAGE / 'build.log').open('w') as log:
        result = subprocess.run(args, stdout=log, stderr=subprocess.STDOUT)
    output = {'completedUtc': datetime.now(timezone.utc).isoformat(), 'exitCode': result.returncode,
              'command': args, 'packages': PACKAGES, 'network': 'none',
              'dependencyInstallation': False, 'liveContainerChanged': False}
    if result.returncode == 0:
        info = json.loads(run(['docker', 'image', 'inspect', staged['candidateTag']], text=True).stdout)[0]
        output['image'] = info['Id']
        for key in ('Entrypoint', 'Cmd', 'Env', 'User', 'WorkingDir'):
            # Compose overrides the image's default Cmd; compare image defaults to image defaults.
            assert info['Config'].get(key) == base['Config'].get(key), key
    save('build-result.json', output)


def stage():
    assert STAGE.is_dir() and not (STAGE / 'staging.json').exists()
    backup = STAGE / 'backup'
    backup.mkdir(mode=0o700)
    info_text = run(['docker', 'inspect', CONTAINER], text=True).stdout
    (backup / 'container-inspect.json').write_text(info_text)
    (backup / 'container-inspect.json').chmod(0o600)
    info = json.loads(info_text)[0]
    assert info['State']['Running'] and not info['Config'].get('OnBuild')
    probe = r'''import os,json
from pathlib import Path
records=[];launches=[]
for name in os.listdir('/proc'):
 if not name.isdigit():continue
 try:
  args=Path('/proc/'+name+'/cmdline').read_bytes().decode().strip('\0').split('\0')
  if not args:continue
  record={'pid':int(name),'args':args,'cwd':os.readlink('/proc/'+name+'/cwd')}
  if len(args)>3 and args[1].endswith('/ros2') and args[2]=='launch' and args[3]=='ur16e_bringup':
   record['environment']=dict(x.split('=',1) for x in Path('/proc/'+name+'/environ').read_bytes().decode().strip('\0').split('\0') if '=' in x);launches.append(record)
  if any(x in ' '.join(args[:2]) for x in ['ros2_control_node','test_task','move_group','default_server_endpoint','robot_state_publisher']):records.append(record)
 except (FileNotFoundError,ProcessLookupError):pass
assert len(launches)==1,launches
print(json.dumps({'launch':launches[0],'processes':records}))'''
    runtime = json.loads(run(['docker', 'exec', CONTAINER, 'python3', '-c', probe], text=True).stdout)
    (backup / 'runtime.json').write_text(json.dumps(runtime, indent=2))
    (backup / 'runtime.json').chmod(0o600)
    (backup / 'host.patch').write_bytes(run(['git', '-C', str(ROOT), 'diff', '--binary', 'HEAD']).stdout)
    dirty = run(['git', '-C', str(ROOT), 'status', '--porcelain'], text=True).stdout
    (backup / 'host-status.txt').write_text(dirty)
    old_head = run(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).stdout.strip()
    source = STAGE / 'source'
    source.mkdir()
    with tarfile.open(STAGE / 'source.tar.gz') as archive:
        for member in archive.getmembers():
            assert (source / member.name).resolve().is_relative_to(source.resolve()), member.name
            assert not member.issym() and not member.islnk(), member.name
        archive.extractall(source)
    old_tag = 'urmir-jazzy:before-fixture-20261102'
    assert subprocess.run(['docker', 'image', 'inspect', old_tag], capture_output=True).returncode != 0
    run(['docker', 'tag', info['Image'], old_tag])
    fresh_build = 'build_codex_fixture_20261102'
    dockerfile = '\n'.join([
        'FROM ' + old_tag,
        'COPY ros_ws/src/ /home/ros_ws/src/',
        'COPY docker/ /home/docker/',
        'RUN source /opt/ros/jazzy/setup.bash && source /home/ros_ws/install/setup.bash && '
        'CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --base-paths /home/ros_ws/src '
        '--build-base /home/ros_ws/' + fresh_build + ' --install-base /home/ros_ws/install '
        '--executor sequential --packages-select ' + ' '.join(PACKAGES) +
        ' --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON',
        '',
    ])
    (source / 'Dockerfile').write_text(dockerfile)
    public_launch = {k: v for k, v in runtime['launch'].items() if k != 'environment'}
    save('staging.json', {'createdUtc': datetime.now(timezone.utc).isoformat(), 'hostRoot': str(ROOT),
                         'hostHead': old_head, 'hostDirtyFiles': dirty.splitlines(),
                         'oldImage': info['Image'], 'oldContainerId': info['Id'], 'oldTag': old_tag,
                         'candidateTag': 'urmir-jazzy:fixture-tiling-20261102',
                         'launch': public_launch, 'processesBefore': runtime['processes'],
                         'sourceArchiveSha256': sha(STAGE / 'source.tar.gz'),
                         'bundleSha256': sha(STAGE / 'fixture-tiling.bundle'),
                         'liveChanged': False, 'robotCommands': False})
    with (STAGE / 'build-driver.log').open('w') as log:
        proc = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), 'build'],
                                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True, close_fds=True)
    print(json.dumps({'staged': True, 'buildPid': proc.pid, 'stage': str(STAGE)}))


if __name__ == '__main__':
    build() if sys.argv[1:] == ['build'] else stage()
