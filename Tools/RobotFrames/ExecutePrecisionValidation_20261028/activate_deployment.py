"""Approved MTC-only activation with source/identity guards and scoped rollback."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

stage = Path('/home/nordbo/.cache/codex/mtc-precision-20261028')
manifest = json.loads((stage / 'manifest.json').read_text())
runtime = json.loads((stage / 'backup/runtime.json').read_text())
container = manifest['container']
root = Path(manifest['host_checkout'])
job = '/tmp/codex-mtc-precision-20261028'
binary = runtime['mtc']['args'][0]
old_pid = runtime['mtc']['pid']
report = {'sources_applied': False, 'old_mtc_stopped': False, 'new_binary_installed': False,
          'rollback': False, 'image_rebuilt': False, 'container_restarted': False,
          'plan_execute_or_io_published': False}

def run(args, **kwargs):
    return subprocess.run(args, check=True, capture_output=True, **kwargs)

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def dex(code, *args):
    return run(['docker', 'exec', container, 'python3', '-c', code, *map(str, args)], text=True).stdout

def cp_in(source, destination):
    run(['docker', 'cp', str(source), container + ':' + destination])

def identities():
    code = '''import json,os,hashlib,sys
out={}
for p in json.loads(sys.argv[1]):
 try:
  s=open('/proc/'+str(p)+'/stat').read()
  out[str(p)]={'start_ticks':s[s.rfind(')')+2:].split()[19],
   'args':open('/proc/'+str(p)+'/cmdline','rb').read().decode().strip('\\0').split('\\0'),
   'cwd':os.readlink('/proc/'+str(p)+'/cwd')}
 except FileNotFoundError:out[str(p)]=None
print(json.dumps(out))'''
    return json.loads(dex(code, json.dumps([p['pid'] for p in manifest['processes_before']])))

def check_unchanged(include_mtc=True):
    current = identities()
    for original in manifest['processes_before']:
        if original['pid'] == old_pid and not include_mtc:
            continue
        actual = current[str(original['pid'])]
        assert actual and actual['start_ticks'] == original['start_ticks'], original['pid']
        assert actual['args'] == original['args'], original['pid']
        if original['pid'] == old_pid:
            assert actual['cwd'] == runtime['mtc']['cwd']
    return current

def probe(require_idle):
    command = "source /opt/ros/jazzy/setup.bash && source /home/ros_ws/install/setup.bash && python3 " + job + '/probe_runtime.py'
    if require_idle:
        command += ' --require-idle'
    result = run(['docker', 'exec', container, 'bash', '--noprofile', '--norc', '-c', command], text=True)
    data = json.loads(result.stdout.strip().splitlines()[-1])
    return data

def atomic_host(path, data):
    temporary = path.with_name(path.name + '.codex-precision-new')
    assert not temporary.exists(), str(temporary)
    temporary.write_bytes(data)
    temporary.chmod(path.stat().st_mode & 0o777)
    os.replace(temporary, path)

def container_replace(source, path, mode=0o644):
    temporary = path + '.codex-precision-new'
    assert dex('import os,sys;print(os.path.exists(sys.argv[1]))', temporary).strip() == 'False'
    cp_in(source, temporary)
    dex('import os,sys;os.chmod(sys.argv[1],int(sys.argv[3]));os.replace(sys.argv[1],sys.argv[2])',
        temporary, path, mode)

def start_mtc():
    run(['docker', 'exec', '-d', '-w', runtime['mtc']['cwd'], container, 'python3',
         job + '/launch_mtc.py', job])

def wait_new(expected_sha):
    script = '''import os,json,hashlib
out=[]
for p in os.listdir('/proc'):
 if not p.isdigit():continue
 try:
  a=open('/proc/'+p+'/cmdline','rb').read().decode().strip('\\0').split('\\0')
  if a and a[0]=='%s':
   e=dict(x.split('=',1) for x in open('/proc/'+p+'/environ','rb').read().decode().strip('\\0').split('\\0') if '=' in x)
   out.append({'pid':int(p),'sha256':hashlib.sha256(open('/proc/'+p+'/exe','rb').read()).hexdigest(),'args':a,'cwd':os.readlink('/proc/'+p+'/cwd'),'environment_sha256':hashlib.sha256(json.dumps(e,sort_keys=True).encode()).hexdigest()})
 except FileNotFoundError:pass
print(json.dumps(out))''' % binary
    deadline = time.monotonic() + 12
    while time.monotonic() < deadline:
        processes = json.loads(dex(script))
        if len(processes) == 1 and processes[0]['pid'] != old_pid:
            assert processes[0]['sha256'] == expected_sha
            expected_args = list(runtime['mtc']['args'])
            for i, value in enumerate(expected_args):
                if value == '--params-file':
                    expected_args[i + 1] = job + '/params/' + Path(expected_args[i + 1]).name
            assert processes[0]['args'] == expected_args
            assert processes[0]['cwd'] == runtime['mtc']['cwd']
            assert processes[0]['environment_sha256'] == hashlib.sha256(
                json.dumps(runtime['mtc']['environment'], sort_keys=True).encode()).hexdigest()
            return processes[0]
        time.sleep(0.25)
    raise RuntimeError('Expected replacement MTC process did not appear')

assert json.loads((stage / 'output/result.json').read_text())['captured_scene_replay'] == 'passed'
assert json.loads((stage / 'runtime-source-comparison.json').read_text())['all_other_runtime_sources_match']
for item in json.loads((stage / 'runtime-source-comparison.json').read_text())['files']:
    data = (stage / 'source' / item['path']).read_bytes().replace(b'\r\n', b'\n')
    assert hashlib.sha256(data).hexdigest() == item['staged_canonical_lf_sha256']
new_binary = stage / 'output/install/lib/ur16e_mtc_gpio_stage/test_task'
new_sha = sha(new_binary)
assert new_sha == (stage / 'output/test_task.sha256').read_text().split()[0]
for item in manifest['source_files']:
    rel = Path(item['path']).relative_to('ros_ws/src/ur16e_mtc_gpio_stage')
    assert sha(stage / 'source' / rel) == item['fixed_sha256']
    assert sha(root / item['path']) == item['baseline_sha256']
check_unchanged()
for path, backup in manifest['container_backups'].items():
    current_sha = dex('import hashlib,sys;print(hashlib.sha256(open(sys.argv[1],"rb").read()).hexdigest())', path).strip()
    assert current_sha == backup['sha256'], path
assert dex('import hashlib,sys;print(hashlib.sha256(open("/proc/"+sys.argv[1]+"/exe","rb").read()).hexdigest())',
           old_pid).strip() == manifest['container_backups'][binary]['sha256']

# Prepare private exact parameter/environment copies before stopping anything.
dex('import os,sys;os.mkdir(sys.argv[1],0o700);os.mkdir(sys.argv[1]+"/params",0o700)', job)
cp_in(stage / 'backup/runtime.json', job + '/runtime.json')
cp_in(stage / 'parent_identity.json', job + '/parent_identity.json')
for helper in ['probe_runtime.py', 'launch_mtc.py']:
    cp_in(stage / helper, job + '/' + helper)
cp_in(new_binary, job + '/test_task.staged')
ldd = dex('''import json,os,subprocess,sys
r=json.load(open(sys.argv[1]))['mtc']
p=subprocess.run(['/usr/bin/ldd',sys.argv[2]],env=r['environment'],capture_output=True,text=True)
assert p.returncode==0 and 'not found' not in p.stdout,p.stdout+p.stderr
print(p.stdout)''', job + '/runtime.json', job + '/test_task.staged')
report['staged_binary_dependencies_resolved'] = True
for path in manifest['parameters_files']:
    backup = manifest['container_backups'][path]
    assert sha(backup['backup']) == backup['sha256']
    current_sha = dex('import hashlib,sys;print(hashlib.sha256(open(sys.argv[1],"rb").read()).hexdigest())', path).strip()
    assert current_sha == backup['sha256'], path
    cp_in(backup['backup'], job + '/params/' + Path(path).name)
before = probe(True)
report['preactivation_probe'] = before
check_unchanged()

try:
    for item in manifest['source_files']:
        rel = Path(item['path']).relative_to('ros_ws/src/ur16e_mtc_gpio_stage')
        staged = stage / 'source' / rel
        atomic_host(root / item['path'], staged.read_bytes())
        container_replace(staged, '/home/ros_ws/src/ur16e_mtc_gpio_stage/' + str(rel))
    header = stage / 'source/include/ur16e_mtc_gpio_stage/plan_validation.hpp'
    container_replace(header, '/home/ros_ws/install/ur16e_mtc_gpio_stage/include/ur16e_mtc_gpio_stage/plan_validation.hpp')
    report['sources_applied'] = True
    # Guard again immediately before the sole signal; no other process is targeted.
    report['immediate_idle_probe'] = probe(True)
    assert report['immediate_idle_probe']['parameters'] == before['parameters']
    check_unchanged()
    stop = '''import os,sys,signal,time
p=int(sys.argv[1]);s=open('/proc/'+str(p)+'/stat').read()
assert s[s.rfind(')')+2:].split()[19]==sys.argv[2]
os.kill(p,signal.SIGINT)
end=time.monotonic()+15
while os.path.exists('/proc/'+str(p)) and time.monotonic()<end:time.sleep(.1)
assert not os.path.exists('/proc/'+str(p)), 'MTC did not exit; no force kill issued'
'''
    dex(stop, old_pid, runtime['mtc']['start_ticks'])
    report['old_mtc_stopped'] = True
    check_unchanged(False)
    container_replace(new_binary, binary, 0o755)
    report['new_binary_installed'] = True
    start_mtc()
    report['new_process'] = wait_new(new_sha)
    report['supervision'] = json.loads(dex('import sys;print(open(sys.argv[1]).read())', job + '/supervisor.json'))
    assert report['supervision']['child_pid'] == report['new_process']['pid']
    assert report['supervision']['original_launch_pid'] == manifest['parent_launch']['pid']
    assert report['supervision']['original_launch_start_ticks'] == manifest['parent_launch']['start_ticks']
    after = probe(False)
    assert after['parameters'] == before['parameters'], 'Replacement model/configuration differs'
    report['postactivation_probe'] = after
    report['processes_after'] = check_unchanged(False)
    report['binary_sha256'] = new_sha
    report['success'] = True
except Exception as error:
    report['error'] = str(error)
    # Do not automatically signal an unknown replacement process. If startup or
    # verification fails, retain backups and require an explicit reviewed rollback.
    report['rollback_required'] = report['old_mtc_stopped']
    (stage / 'activation-result.json').write_text(json.dumps(report, indent=2))
    raise
(stage / 'activation-result.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report))
