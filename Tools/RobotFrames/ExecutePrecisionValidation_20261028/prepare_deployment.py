"""Approved Nordbo staging/backups only; does not modify sources or processes."""
import base64
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

cfg = json.load(sys.stdin)
root = Path('/home/nordbo/Unity/UR16UnityControlV2/UR16UnityControl')
stage = Path('/home/nordbo/.cache/codex/mtc-precision-20261028')
container = 'urmir-jazzy'
env = dict(os.environ, GIT_OPTIONAL_LOCKS='0')

def run(args, **kwargs):
    return subprocess.run(args, check=True, capture_output=True, **kwargs)

def digest(data):
    return hashlib.sha256(data).hexdigest()

assert run(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True, env=env).stdout.strip() == cfg['head']
assert not run(['git', '-C', str(root), 'status', '--porcelain'], text=True, env=env).stdout.strip()
assert not stage.exists(), 'Refusing to overwrite an existing deployment stage'
for entry in cfg['files']:
    assert digest((root / entry['path']).read_bytes()) == entry['baseline_sha256'], entry['path']
    assert digest(base64.b64decode(entry['data'])) == entry['fixed_sha256'], entry['path']

probe = r'''
import hashlib,json,os
records=[]
for item in os.listdir('/proc'):
 if not item.isdigit():continue
 try:
  raw=open('/proc/'+item+'/cmdline','rb').read();args=raw.decode().strip('\0').split('\0')
  if not args or not any(x in args[0] for x in ['ros2_control_node','dashboard_client','robot_state_helper','controller_stopper','urscript_interface','robot_state_publisher','trajectory_until','ur16e_commander','ur16e_environment_updater','test_task','move_group','rviz2']):
   if not any(a.endswith('/ros_tcp_endpoint/default_server_endpoint') for a in args):continue
  stat=open('/proc/'+item+'/stat').read();ticks=stat[stat.rfind(')')+2:].split()[19]
  records.append({'pid':int(item),'args':args,'start_ticks':ticks,'exe':os.readlink('/proc/'+item+'/exe')})
 except (FileNotFoundError,ProcessLookupError):pass
mtc=[x for x in records if x['args'][0].endswith('/ur16e_mtc_gpio_stage/test_task')]
assert len(mtc)==1,mtc
p=mtc[0]['pid'];mtc[0]['cwd']=os.readlink('/proc/'+str(p)+'/cwd')
environ=open('/proc/'+str(p)+'/environ','rb').read()
mtc[0]['environment']=dict(x.split('=',1) for x in environ.decode().strip('\0').split('\0') if '=' in x)
print(json.dumps({'processes':records,'mtc':mtc[0]}))
'''
runtime = json.loads(run(['docker', 'exec', container, 'python3', '-c', probe], text=True).stdout)
assert runtime['mtc']['pid'] == cfg['expected_pid']
stage.mkdir(parents=True, mode=0o700)
(stage / 'backup').mkdir()
(stage / 'output').mkdir()
runtime_path = stage / 'backup/runtime.json'
runtime_path.write_text(json.dumps(runtime, indent=2))
runtime_path.chmod(0o600)
package = root / 'ros_ws/src/ur16e_mtc_gpio_stage'
shutil.copytree(package, stage / 'source')
for entry in cfg['files']:
    relative = Path(entry['path']).relative_to('ros_ws/src/ur16e_mtc_gpio_stage')
    target = stage / 'source' / relative
    target.write_bytes(base64.b64decode(entry['data']))
    host_backup = stage / 'backup/host' / relative
    host_backup.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(root / entry['path'], host_backup)

backup_paths = [
    '/home/ros_ws/src/ur16e_mtc_gpio_stage/include/ur16e_mtc_gpio_stage/plan_validation.hpp',
    '/home/ros_ws/src/ur16e_mtc_gpio_stage/test/test_plan_validation.cpp',
    '/home/ros_ws/install/ur16e_mtc_gpio_stage/include/ur16e_mtc_gpio_stage/plan_validation.hpp',
    '/home/ros_ws/install/ur16e_mtc_gpio_stage/lib/ur16e_mtc_gpio_stage/test_task',
]
args = runtime['mtc']['args']
params = [args[i + 1] for i, arg in enumerate(args) if arg == '--params-file']
assert len(params) == 9
backups = {}
for path in backup_paths + params:
    target = stage / 'backup/container' / path.lstrip('/')
    target.parent.mkdir(parents=True, exist_ok=True)
    run(['docker', 'cp', container + ':' + path, str(target)])
    backups[path] = {'sha256': digest(target.read_bytes()), 'backup': str(target)}
assert backups[backup_paths[0]]['sha256'] == cfg['files'][0]['baseline_sha256']
assert backups[backup_paths[2]]['sha256'] == cfg['files'][0]['baseline_sha256']
assert backups[backup_paths[3]]['sha256'] == cfg['expected_binary_sha256']

for entry in cfg['helpers']:
    target = stage / entry['path']
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(base64.b64decode(entry['data']))
    assert digest(target.read_bytes()) == entry['sha256']

manifest = {
    'stage': str(stage), 'host_checkout': str(root), 'container': container,
    'image': json.loads(run(['docker', 'inspect', container], text=True).stdout)[0]['Image'],
    'head': cfg['head'], 'source_files': [{k:v for k,v in e.items() if k != 'data'} for e in cfg['files']],
    'runtime': {k:v for k,v in runtime['mtc'].items() if k != 'environment'},
    'runtime_backup': str(runtime_path), 'processes_before': runtime['processes'],
    'container_backups': backups, 'parameters_files': params,
    'sources_applied': False, 'production_install_changed': False, 'processes_signalled': False,
}
# Keep environment private in the backup; public manifest records identities only.
for record in manifest['processes_before']:
    record.pop('environment', None)
(stage / 'manifest.json').write_text(json.dumps(manifest, indent=2))
print(json.dumps({'prepared': True, 'stage': str(stage), 'manifest': manifest}))
