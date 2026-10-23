# Decisions

## 2026-10-22: Tool previews and TCP-contact pick/place

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

## 2026-10-22: Plan exact TCP A/B poses before execution

- Owner clarified that captured A/B markers are exact TCP poses and must produce a reviewable plan before any execution. Approval: “Apply the patch and build/test it,” covering the reviewed backend patch, API/project notes and offline package validation.
- Use separate plan-only, preview/status and identified execution topics. Preserve full target orientation and the actual startup `tcp` frame; do not reinterpret TCP poses as tile surfaces or send them to the immediate basic-motion topics.
- Require matching request token, successful status and visible preview in Unity. The backend checks frozen mode/ID under its command mutex before consuming one execution attempt and retaining the existing scene/start-state preflight. TCP execution forbids IO; the existing tile path keeps its vacuum capability requirement.
- This approval covers source/docs and offline validation only. Deployment, launching the real stack and robot/IO operation remain outside scope.

## 2026-10-22: Integrate Safe&Light as a startup tool

- Owner approved making `kenos_sl` usable throughout the toolchain and explicitly requested a dedicated `urmir_rsp_kenos_sl.launch.py` wrapper selected by `start_robot.launch.py`.
- Preserve startup-only selection and existing `none`/`kenos` behavior. Register a separate SRDF group and contact links for Safe&Light; allow vacuum tasks only after matching model validation.
- Owner confirmed the existing vacuum binding: `/io_and_status_controller/set_io`, function 1, digital output 3, on/off states 1/0 and the same three-second release delay.
- Expose five separate `kenos_sl_*` mounting/TCP arguments consistently at bringup, control and planner entry points. Keep the FlexiGrip TCP arguments separate so they cannot silently replace the eccentric Safe&Light working point.
- Keep the initial mounting estimates visible as estimates. Integration does not establish physical calibration, payload or commissioning; no hardware operation or deployment is authorized by this change.

## 2026-10-22: Prepare a reusable Safe&Light description

- Owner approved creating `kenos_sl.urdf.xacro` in the existing tool description package, using the sibling macro's parent/prefix pattern and a fixed `tcp` working frame. Full launch/SRDF/task registration is separate work.
- The owner supplied `KSL.240.120.N241.CVL.S1.1STH.X.stp` and photos identifying OnRobot QC-RS v3 item 109498 and QC-T v2 item 102014, with an additional black adapter.
- STEP assembly analysis established 240 x 120 x 24 mm foam, a 250 x 130 mm holder envelope, and a nominal face centre 34 mm along the gripper from the mounting-hole centre and 89.75 mm outward from its mounting face.
- Use primitive collision and visual envelopes, including the ejector and air fitting. Approximate the complete changer/adapter stack with a cylinder. The owner measured approximately 150 mm installed axial reach, giving an initial stack height of 60.25 mm; the 50 mm cylinder radius and zero clocking are unverified assumptions exposed as parameters.
- Preserve the eccentric offset when adjusting mounting yaw. No calibrated physical TCP, payload, mass/inertia, flexible-hose envelope or vacuum IO binding is implied by this reusable model.

## 2026-09-14: Gate vacuum tasks on the mounted tool's capabilities

- Owner instruction: "i think there is one P1 remaining? if so, fix it please", referring to no-tool selection still building Kenos vacuum stages.
- Pass the same selected `tool` into MTC behavior and model generation. Resolve a small immutable definition for `none` or `kenos`, including TCP and optional vacuum collision/IO settings. This is not a runtime plugin or tool-changing framework.
- Validate the selected definition against the startup model before command subscriptions exist. Unknown selections, incompatible models, missing required geometry/groups or a different IK frame fail configuration. Keep `tool` and `ik_frame` read-only after startup and reuse the validated model for new tasks.
- With `none`, retain basic pose motion and explicit scene setup. Reject pick/place and execution before task scene changes, planning or executor construction; clear prepared-plan eligibility, preview and planning success. Report a defined unsupported-capability rejection.
- Kenos task construction and custom execution use the same vacuum binding; preserve pin 3, function 1, on/off states and three-second release delay. Planning remains marker-only and does not require a live IO service. Broader deployment wiring configuration, typed operations, feedback and recovery remain follow-up work.
- Preserve the earlier uncommitted P1 fixes. No robot or container operation is authorized by this correction.

## 2026-09-14: Preserve frame and TCP meaning in basic motion

- Owner instruction: "ok go", approving the explanation of the fourth P1.
- Introduce explicit `tcp_pose_target` and `tcp_pose_array_target` contracts. Preserve stamped frame/time semantics, validate all input poses before motion and resolve one frame snapshot per request. Arrays remain sequential planned moves and stop on the first planning/execution failure.
- Preserve unmodified clients through an explicit startup `enable_legacy_pose_topics` opt-in, disabled by default. Legacy topics retain their old per-endpoint frame and wrist meaning; convert wrist goals through the selected model's fixed wrist-to-TCP transform into the same TCP execution path.
- Separate topic names prevent silently interpreting an existing valid wrist command as a tool-tip command. Document migration for Unity and retain the example publishers' original wrist goals. The standalone `test_moveit` demo explicitly names its legacy wrist endpoint.
- Publish indexed motion status without inventing command IDs or cancellation. Startup parameters are read-only; tool selection remains a startup operation. No physical operation is authorized by this correction; the existing third P1 changes remain uncommitted alongside it.

## 2026-09-07: Apply the Buildwise development framework

- Context: the user requested using `buildwise-be/BW_CODEX_DEV_GUIDE` before a repository architecture review.
- Action: initialize framework 2.0.1 and project memory; record repository-derived facts in `docs/wiki/` and proposed improvements in `docs/ai-context/`.
- This is a review mission. No application architecture proposal is an approved implementation decision.
- Historical product and engineering rationale is undocumented; inferred rationale must be labeled as inference.

## 2026-09-07: Select the mounted tool at startup

- Owner clarification: "Select the mounted tool at startup" in response to the tool workflow question.
- Consequence for this review: propose one immutable resolved tool selection for the session. Runtime tool switching and automatic changers are outside the design target.
- This constrains the recommendations; it does not approve a specific refactor or tool-profile schema.

## 2026-09-09: Fix the first P1 model-input disagreement

- Owner instruction: "Please fix that first P1", following the explanation of separate URDF/SRDF inputs.
- Scope: explicitly supply tool, UR type and arm calibration to MoveGroup/MTC URDF generation, and forward calibration through main bringup and control.
- Keep the existing launch structure and startup-selected tool workflow. A shared tool-profile architecture and the other P1 changes remain separate work.
- Add model-consistency regression coverage and document standalone launch calibration arguments. No physical operation is part of this fix.

## 2026-09-10: Preserve scene effects during custom execution

- Owner instruction: "You can now solve the second P1".
- Scope: apply each serialized MTC scene effect after successful motion or IO, including scene-only stages; preserve the existing GPIO marker contract and settle delay.
- Use acknowledged, bounded `ApplyPlanningScene` service calls and stop on operation or scene-update failure. Preserve completed effects and vacuum output; do not attempt automatic rollback or release. Timeout is an unknown outcome requiring inspection before retrying.
- Preserve MTC's full-scene/diff semantics and attached objects while removing planned joint-state updates. Add portable sequencing tests and actual MoveIt scene regression tests; do not operate hardware.
- Typed IO commands, cached-plan validation, execution ownership and automated recovery remain separate work.

## 2026-09-10: Require a valid, unconsumed plan before execution

- Owner instruction: "Solve next P1", referring to the third review finding.
- Use explicit Idle/Planning/Planned/Executing/Faulted admission states and reject overlapping command callbacks with a task mutex. Preserve the existing Bool command; false remains a no-op, not cancellation.
- Freeze the chosen serialized solution and its actual start scene after successful planning. Invalidate prior readiness before new planning or scene setup, and consume it before any execution attempt. Failed attempts and completed tasks require a new plan.
- Guard task construction, model loading, initialization and planning; clear retained previews on invalidation/failure/consumption. Add execution/readiness status separately from the existing planning-status JSON.
- Check the current full scene and recent joint feedback before first operation. Default joint tolerance is 0.01 rad for this UR arm; feedback receipt age is limited to one second. Compare geometry conservatively after ignoring timestamps/ordering and noncollision metadata.
- The check is not exclusive ownership of the robot/scene, proof of physical tool identity, or an arrival-time command/plan identity protocol. External writers, delayed Bool commands, cancellation and automated recovery remain follow-up work. No hardware operation is authorized by this fix.
