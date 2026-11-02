"""Add the validated test-only commit without rebuilding unchanged runtime binaries."""
import json
from pathlib import Path
import shutil
import subprocess

STAGE = Path('/home/nordbo/.cache/codex/fixture-tiling-20261102')
COMMIT = '6ec8146f99ab7ef670b45e39a203e35de6bbc295'
TEST = 'ros_ws/src/ur16e_bringup/test/test_model_consistency.py'


def run(args):
    return subprocess.run(args, check=True, capture_output=True, text=True).stdout


def main():
    source = json.loads((STAGE / 'source-sync.json').read_text())
    checkout = Path(source['activeSourceCheckout'])
    assert not run(['git', '-C', str(checkout), 'status', '--porcelain']).strip()
    run(['git', '-C', str(checkout), 'fetch', str(STAGE / 'fixture-tiling-6ec8146.bundle'), 'refs/heads/codex/fixture-tiling'])
    assert run(['git', '-C', str(checkout), 'diff', '--name-only', source['commit'], COMMIT]).splitlines() == [TEST]
    run(['git', '-C', str(checkout), 'merge', '--ff-only', 'FETCH_HEAD'])
    assert run(['git', '-C', str(checkout), 'rev-parse', 'HEAD']).strip() == COMMIT
    original = json.loads((STAGE / 'build-result.json').read_text())
    shutil.copy2(STAGE / 'build-result.json', STAGE / 'build-result-runtime-131ea4e.json')
    run(['docker', 'tag', original['image'], 'urmir-jazzy:fixture-runtime-131ea4e'])
    context = STAGE / 'test-fix-image'
    context.mkdir()
    shutil.copy2(checkout / TEST, context / 'test_model_consistency.py')
    (context / 'Dockerfile').write_text('FROM urmir-jazzy:fixture-runtime-131ea4e\nCOPY test_model_consistency.py /home/' + TEST + '\n')
    tag = 'urmir-jazzy:fixture-tiling-20261102'
    with (STAGE / 'test-fix-build.log').open('w') as log:
        subprocess.run(['docker', 'build', '--network', 'none', '--pull=false', '--progress=plain', '-t', tag, str(context)],
                       check=True, stdout=log, stderr=subprocess.STDOUT)
    image = json.loads(run(['docker', 'image', 'inspect', tag]))[0]['Id']
    binary = '/home/ros_ws/install/ur16e_mtc_gpio_stage/lib/ur16e_mtc_gpio_stage/test_task'
    def digest(image_id):
        return run(['docker', 'run', '--rm', '--network', 'none', '--entrypoint', 'sha256sum', image_id, binary]).split()[0]
    old_hash, new_hash = digest(original['image']), digest(image)
    assert old_hash == new_hash
    original.update({'image': image, 'sourceCommit': COMMIT, 'compiledRuntimeImage': original['image'],
                     'testOnlyLayer': TEST, 'unchangedMtcBinarySha256': new_hash})
    (STAGE / 'build-result.json').write_text(json.dumps(original, indent=2) + '\n')
    source.update({'commit': COMMIT, 'testOnlyFollowup': True, 'githubPushed': True})
    (STAGE / 'source-sync.json').write_text(json.dumps(source, indent=2) + '\n')
    (STAGE / 'test-result.json').rename(STAGE / 'test-result-first.json')
    (STAGE / 'test-output').rename(STAGE / 'test-output-first')
    subprocess.run(['python3', str(STAGE / 'test_image.py')], check=True)


if __name__ == '__main__':
    main()
