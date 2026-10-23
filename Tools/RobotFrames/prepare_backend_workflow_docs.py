"""Stage concise repository/API updates for the authorized workflow changes."""
from pathlib import Path
import json
SOURCE=Path('C:/Users/aca/source/UR16UnityControl')
OUT=Path(__file__).parent/'BackendWorkflowPatch'
changed=[]
def read(path):return (SOURCE/path).read_text(encoding='utf-8-sig')
def put(path,text):
    destination=OUT/path;destination.parent.mkdir(parents=True,exist_ok=True)
    destination.write_text(text,encoding='utf-8',newline='\n');changed.append(path)
def insert_before_refresh(text,addition):
    return text.replace('\n## Refresh\n','\n'+addition+'\n## Refresh\n',1)

p='docs/wiki/API.md';text=read(p)
row='| `/execute_task` |'
assert text.count(row)==1
index=text.index(row)
text=text[:index]+'| `/pick_place_tcp_target` | `ur16e_interfaces/PickPlacePoses` | MTC node | Plan-only desired pick/place TCP contacts; position and outward tool +Z are fixed, twist is sampled at both ends. Requires vacuum capability. |\n'+text[index:]
addition='''## Tool-contact pick/place targets

`/pick_place_tcp_target` is an additive replacement for XR clients that place
tool references. The read-only startup bool `pick_place_tcp_planning_enabled`
defaults to `true`; clients must check it before publishing. Both fields of
`PickPlacePoses` must name `ur_base_link`, contain finite positions and nonzero
finite quaternions. The backend normalizes rotations and treats the contacts as
static base-relative poses, clearing their timestamps.

Each position is the desired tool TCP contact point, including any deliberate
client depth adjustment. Tool +Z points outward through the suction foam toward
the object. Pick and place keep that normal while independently sampling twelve
tool twists at 30-degree intervals. The returned trajectory shows the chosen
clocking. These samples are finite alternatives, not continuous yaw optimization.

The planning tile remains 100 x 100 x 4 mm. Its centre is contact position plus
half its thickness along tool +Z; the offset rotates with the supplied tool
orientation. The pick pregrasp is 100 mm behind the contact along tool -Z, and
the contact approach advances along +Z. Place candidates directly address TCP,
so an attached object's arbitrary grasp twist does not change the requested
place point/normal. Client +5/-5 mm depth steps shift the desired TCP itself;
the backend adds no unrequested compression, force control or compliance.

This endpoint uses the existing `/mtc/planning_status`,
`/mtc/planned_trajectory`, `/mtc/execution_status` and separate
`/execute_task=true` vacuum execution flow. These legacy outputs have no request
ID. `/pick_place_poses_target` retains its original surface/object convention;
renaming that topic without changing pose meaning is incorrect.
'''
text=insert_before_refresh(text,addition);put(p,text)

p='docs/wiki/TASK_EXECUTION.md';text=read(p)
addition='''## TCP contact targets and execution-frame correction (2026-10-22)

The additive `pick_place_tcp_target` callback enters the same vacuum planning
gate with a different, validated input contract: desired TCP contact positions
and outward +Z normals. It derives the planning object's centre along the tool
normal rather than reference-frame vertical. Grasp and place each sample twelve
local-Z twists; place uses monitored `GeneratePose`/`ComputeIK` alternatives
directly for TCP, leaving the attached object to follow its rigid grasp transform.
The original surface callback and its historical object placement remain intact.
See [the contact wire contract](API.md#tool-contact-pickplace-targets).

Execution validation now uses MoveIt's actual transform serialization: source
is `header.frame_id`, while multiple entries legitimately share planning-target
`child_frame_id`. Empty names and duplicate sources still reject execution.
After structural validation, only transform sources referenced by actual world
objects, attachments or nonempty octomap geometry participate in exact scene
comparison. Unused measured-controller TCP transforms can vary with encoder
feedback without invalidating unchanged collision geometry. Referenced frame,
geometry, permissions, padding, scaling and joint checks remain protected.

Before a TCP-only plan, the node can remove its own unchanged, unexecuted
synthetic task-preview object. It records the acknowledged object's serialized
world geometry after creating it; changed external geometry rejects cleanup.
Manual scene objects, attached objects, objects from a previous node session and
objects after any vacuum execution attempt are retained. No shared scene lock
or external ownership protocol is added; external writers can still race a
point-in-time check.
'''
text=insert_before_refresh(text,addition);put(p,text)

p='docs/ai-context/DECISIONS.md';text=read(p)
addition='''## 2026-10-22: Tool previews and TCP-contact pick/place

- Owner requested fixing `invalid fixed frame names`, replacing XR point/surface
  references with the selected tool, and defining pick/place as tool contact
  points with twist freedom and optional 5 mm local-Z depth adjustments.
- Add a distinct contact topic/capability instead of reinterpreting the existing
  surface topic. Preserve separate preview and Execute; sample tool +Z twist at
  pick and place without changing their specified positions or normals.
- Correct MoveIt's source/target transform convention in preflight and ignore
  only transforms unused by collision scene components. Preserve relevant
  scene/start-state checks. Remove only unchanged, unexecuted task-preview
  geometry created by the current process when returning to TCP-only planning.
- This covers source/docs and offline validation. No deployment, real-stack
  restart, motion, vacuum IO or physical calibration is authorized by this change.

'''
text=text.replace('# Decisions\n\n','# Decisions\n\n'+addition,1);put(p,text)

p='docs/ai-context/CURRENT_STATE.md';text=read(p)
addition='''- Owner-requested contact workflow and execution-validator correction are
  implemented locally (2026-10-22): `pick_place_tcp_target` has an explicit
  tool-contact/+Z contract and independent twist samples at pick/place. The
  scene validator accepts MoveIt's shared target frame and excludes unused
  controller-feedback transforms from collision-scene comparison. Offline
  verification is recorded in [build/testing](../wiki/BUILD_AND_TESTING.md).
  No Nordbo deployment, real-stack restart or hardware command was performed.

'''
text=text.replace('# Current State\n\n','# Current State\n\n'+addition,1);put(p,text)

p='docs/ai-context/KNOWN_ISSUES.md';text=read(p)
addition='''- Contact-target and execution-frame fixes are local source changes until
  deployed by an authorized operator. Pick/place yaw uses twelve 30-degree
  alternatives, and legacy vacuum result/execution messages still have no
  request IDs. Depth offsets are positional contact commands, not force control
  or proof of actual foam compression. Synthetic preview ownership is tracked
  only within one task-node process; unknown/manual/attached/executed scene
  objects are intentionally preserved. Hardware behavior remains unverified.

'''
text=text.replace('# Known Issues\n\n','# Known Issues\n\n'+addition,1);put(p,text)

p='docs/ai-context/CHANGELOG_AI.md';text=read(p)
addition='''- Corrected execution transform validation to use MoveIt source frame keys
  and compare only collision-referenced transforms. Added the separately
  advertised tool-contact pick/place endpoint with rotated object offsets and
  independent pick/place twist sampling; preserved the old surface API. Added
  source/target and encoder-jitter regressions plus contact geometry tests.
  Offline package build/test results are in build/testing; no deployment or
  hardware operation was performed.

'''
text=text.replace('## 2026-10-22\n\n','## 2026-10-22\n\n'+addition,1);put(p,text)

manifest=json.loads((OUT/'manifest.json').read_text())
manifest+=changed
(OUT/'manifest.json').write_text(json.dumps(list(dict.fromkeys(manifest)),indent=2),encoding='utf-8')
print(json.dumps(changed,indent=2))
