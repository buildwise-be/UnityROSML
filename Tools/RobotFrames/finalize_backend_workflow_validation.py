"""Refresh review copies and proof records after the authorized offline checks."""
import hashlib
import json
from pathlib import Path
import subprocess

SOURCE = Path('C:/Users/aca/source/UR16UnityControl')
ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'BackendWorkflowPatch'

changes = {
    'docs/wiki/BUILD_AND_TESTING.md': [
        ('outputs were isolated from the repository. No application node, controller,\nreal-stack launch, robot connection, motion or IO was run.',
         'outputs were isolated from the repository. No controller, real-stack launch,\nrobot connection, hardware motion or IO was run.'),
        ('The source adds independent sampled place TCP goals; compilation and focused\ngeometry tests do not establish actual MoveIt task planning, ROS-TCP delivery,\nphysical compression, payload, collision-model fidelity or hardware execution.\nThe Nordbo deployment is unchanged. No lint target is registered in the package.',
         'A subsequent isolated integration smoke started the new application executable\nwith the captured live URDF/SRDF and joint snapshot, installed KDL/OMPL and joint\nlimits, in-memory Get/ApplyPlanningScene services and static joint feedback.\nThe separately advertised contact endpoint produced a full MTC solution and a\n74-point preview, including the new independent place-twist alternatives.\nSwitching to exact TCP A/B then removed only the same node\'s unchanged,\nunexecuted preview object and produced an 11-point preview; the ground object\nremained. Both requests were plan-only inside `--network none`, with no Execute\nrequest, driver or real ROS stack. Logs and JSON proof are stored in the Unity\nproject\'s `Tools/RobotFrames/BackendWorkflowBuild/` directory.\n\nThese checks do not establish ROS-TCP delivery, physical compression, payload,\ncollision-model fidelity or hardware execution. The Nordbo deployment is\nunchanged. No lint target is registered in the package.')],
    'docs/ai-context/CURRENT_STATE.md': [
        ('verification is recorded in [build/testing](../wiki/BUILD_AND_TESTING.md).',
         'verification is recorded in [build/testing](../wiki/BUILD_AND_TESTING.md):\n  the actual new node produced contact and exact-TCP preview plans against\n  local mocked scene services in a network-disabled container.')],
    'docs/ai-context/CHANGELOG_AI.md': [
        ('  Offline package build/test results are in build/testing; no deployment or\n  hardware operation was performed.',
         '  Offline package build/test and actual-node mock-scene smoke results are\n  in build/testing: contact and subsequent exact-TCP plans produced previews,\n  with no Execute request, deployment or hardware operation.')],
    'docs/ai-context/KNOWN_ISSUES.md': [
        ('ROS-TCP transport, physical planning/alignment and execution remain unverified; no deployment or application launch was performed.',
         'ROS-TCP transport, physical planning/alignment and execution remain unverified; no deployment or real-stack application launch was performed. The later network-isolated mock-scene smoke produced contact and exact-TCP preview plans.')],
}

for relative, replacements in changes.items():
    text = (SOURCE / relative).read_text(encoding='utf-8-sig')
    for old, new in replacements:
        if old in text:
            text = text.replace(old, new, 1)
        else:
            assert new in text, relative
    target = OUT / relative
    target.write_text(text, encoding='utf-8', newline='\n')

if '--apply-docs' in __import__('sys').argv:
    for relative in changes:
        (SOURCE / relative).write_text((OUT / relative).read_text(), encoding='utf-8', newline='\n')

if '--proof' in __import__('sys').argv:
    import difflib
    paths = json.loads((OUT / 'manifest.json').read_text())
    patch = []
    for relative in paths:
        current = (SOURCE / relative).read_text(encoding='utf-8-sig')
        (OUT / relative).write_text(current, encoding='utf-8', newline='\n')
        baseline = subprocess.run(['git', '-C', str(SOURCE), 'show', 'HEAD:' + relative], capture_output=True, check=False)
        original = baseline.stdout.decode('utf-8-sig') if baseline.returncode == 0 else ''
        patch.extend(difflib.unified_diff(original.splitlines(keepends=True), current.splitlines(keepends=True),
                                        fromfile='a/' + relative if baseline.returncode == 0 else '/dev/null', tofile='b/' + relative))
    (OUT / 'backend-workflow.patch').write_text(''.join(patch), encoding='utf-8')
    hashes = {relative: hashlib.sha256((SOURCE / relative).read_bytes()).hexdigest() for relative in paths}
    build = ROOT / 'BackendWorkflowBuild'
    smoke = json.loads((build / 'contact_smoke.json').read_text())
    proof = {'date': '2026-10-22', 'repository': str(SOURCE), 'branch': 'codex/architecture-review',
             'source_sha256': hashes, 'configure_exit': 0, 'build_exit': 0,
             'ctest_targets_passed': 8, 'ctest_targets_total': 9,
             'plan_validation_cases_passed': 9, 'contact_cases_passed': 4,
             'known_baseline_failure': 'SceneEffectsTest.SerializedPickPlaceEffectsUpdateTheLiveSceneInOrder at test_scene_effects.cpp:166',
             'offline_contact_full_solution': smoke['full_solution'],
             'offline_contact_preview_points': max(smoke['preview_trajectory_points']),
             'offline_tcp_after_contact_full_solution': any(status.get('overall_success') for status in smoke['tcp_after_contact']['statuses']),
             'offline_tcp_preview_points': max(smoke['tcp_after_contact']['preview_points']),
             'offline_cleanup_preserved': smoke['tcp_after_contact']['remaining_world_objects'],
             'hardware': False, 'deployment': False, 'container_network': 'none', 'execute_request_published': False,
             'committed': False, 'pushed': False}
    (ROOT / 'new_workflow_backend_validation_summary.json').write_text(json.dumps(proof, indent=2))
    print(json.dumps(proof, indent=2))
