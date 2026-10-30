"""Approved package-only image layer; never restarts/recreates the live container."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

stage = Path('/home/nordbo/.cache/codex/mtc-precision-20261028')
manifest = json.loads((stage / 'manifest.json').read_text())
activation = json.loads((stage / 'activation-result.json').read_text())
assert activation['success']
base = manifest['image']
alias = 'urmir-jazzy:latest'
backup = 'urmir-jazzy:before-mtc-precision-20261028'
fixed = 'urmir-jazzy:mtc-precision-20261028'

def run(args):
    return subprocess.run(args, check=True, capture_output=True, text=True).stdout

def image(name):
    result = subprocess.run(['docker', 'image', 'inspect', name], capture_output=True, text=True)
    return json.loads(result.stdout)[0] if result.returncode == 0 else None

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

old = image(alias)
assert old and old['Id'] == base, 'Compose image alias changed; refusing to overwrite another build'
assert not old['Config'].get('OnBuild'), 'Unexpected inherited build actions'
assert image(backup) is None and image(fixed) is None, 'Refusing to overwrite deployment image tags'
context = stage / 'image-context'
assert not context.exists()
context.mkdir()
sources = {
    'test_task': stage / 'output/install/lib/ur16e_mtc_gpio_stage/test_task',
    'plan_validation.hpp': stage / 'source/include/ur16e_mtc_gpio_stage/plan_validation.hpp',
    'test_plan_validation.cpp': stage / 'source/test/test_plan_validation.cpp',
}
assert sha(sources['test_task']) == activation['binary_sha256']
approved = {entry['path']: entry['fixed_sha256'] for entry in manifest['source_files']}
assert sha(sources['plan_validation.hpp']) == approved['ros_ws/src/ur16e_mtc_gpio_stage/include/ur16e_mtc_gpio_stage/plan_validation.hpp']
assert sha(sources['test_plan_validation.cpp']) == approved['ros_ws/src/ur16e_mtc_gpio_stage/test/test_plan_validation.cpp']
expected = {}
for name, source in sources.items():
    shutil.copy2(source, context / name)
    (context / name).chmod(0o755 if name == 'test_task' else 0o644)
    expected[name] = sha(context / name)
run(['docker', 'tag', base, backup])
assert image(backup)['Id'] == base
dockerfile = '\n'.join([
    'FROM ' + backup,
    'LABEL org.buildwise.mtc-precision-fix="2026-10-28"',
    'COPY test_task /home/ros_ws/install/ur16e_mtc_gpio_stage/lib/ur16e_mtc_gpio_stage/test_task',
    'COPY plan_validation.hpp /home/ros_ws/src/ur16e_mtc_gpio_stage/include/ur16e_mtc_gpio_stage/plan_validation.hpp',
    'COPY plan_validation.hpp /home/ros_ws/install/ur16e_mtc_gpio_stage/include/ur16e_mtc_gpio_stage/plan_validation.hpp',
    'COPY test_plan_validation.cpp /home/ros_ws/src/ur16e_mtc_gpio_stage/test/test_plan_validation.cpp',
    '',
])
(context / 'Dockerfile').write_text(dockerfile)
build_log = run(['docker', 'build', '--network', 'none', '--pull=false', '-t', fixed, str(context)])
(stage / 'image-build.log').write_text(build_log)
new = image(fixed)
assert new and new['Id'] != base
for key in ['Entrypoint', 'Cmd', 'Env', 'User', 'WorkingDir']:
    assert new['Config'].get(key) == old['Config'].get(key), key
verification = stage / 'image-verification'
verification.mkdir()
container_id = run(['docker', 'create', '--name', 'codex-mtc-precision-image-verify', fixed, '/bin/true']).strip()
try:
    checked = {}
    for name, path in {
        'binary': '/home/ros_ws/install/ur16e_mtc_gpio_stage/lib/ur16e_mtc_gpio_stage/test_task',
        'source_header': '/home/ros_ws/src/ur16e_mtc_gpio_stage/include/ur16e_mtc_gpio_stage/plan_validation.hpp',
        'installed_header': '/home/ros_ws/install/ur16e_mtc_gpio_stage/include/ur16e_mtc_gpio_stage/plan_validation.hpp',
        'test_source': '/home/ros_ws/src/ur16e_mtc_gpio_stage/test/test_plan_validation.cpp',
    }.items():
        target = verification / name
        run(['docker', 'cp', container_id + ':' + path, str(target)])
        checked[name] = sha(target)
    assert checked['binary'] == expected['test_task']
    assert checked['source_header'] == checked['installed_header'] == expected['plan_validation.hpp']
    assert checked['test_source'] == expected['test_plan_validation.cpp']
finally:
    run(['docker', 'rm', container_id])
assert image(alias)['Id'] == base, 'Compose image alias changed during verification; refusing to retag'
assert image(backup)['Id'] == base
run(['docker', 'tag', new['Id'], alias])
assert image(alias)['Id'] == new['Id']
live = json.loads(run(['docker', 'inspect', 'urmir-jazzy']))[0]
assert live['Image'] == base and live['State']['Running']
result = {'old_image': base, 'backup_tag': backup, 'fixed_tag': fixed, 'compose_alias': alias,
          'new_image': new['Id'], 'verified_files': checked,
          'live_container_base_unchanged': True, 'live_container_restarted': False,
          'live_container_recreated': False, 'verification_container_started': False,
          'dependency_or_workspace_rebuild': False, 'registry_push': False}
(stage / 'image-result.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result))
