# Nordbo fixture and tool-model deployment — 2026-11-02

Deployment succeeded. Nordbo runs image
`sha256:a13990f26a3ef63e257431a63c70a60c2795ddb966d669b79de41d8eb8da91b2`
with runtime source commit `6ec8146f99ab7ef670b45e39a203e35de6bbc295`.
Driver/RSP, MoveGroup, MTC and ROS-TCP endpoint were restarted together; the final
passive probe reports idle, fresh joint feedback and matching compared models,
with the 55.25 mm tool attachment in all three model descriptions. The new Python
fixture message/service imports work in the deployed container.

All ten local application packages built. Of 111 reported test cases, 110 pass;
the sole failure is the independently known unchanged
`SceneEffectsTest.SerializedPickPlaceEffectsUpdateTheLiveSceneInOrder`
tile/hand collision-matrix assertion. Both isolated two-tile planning replays,
with and without per-tile Home, pass. The initial candidate's launch-test helper
assumption and original failing result are retained separately; the final image
includes its test-only correction without changing the compiled MTC executable.

Branch `codex/fixture-tiling` was pushed to GitHub and synchronized into
`/home/nordbo/Unity/UR16UnityControlV2/UR16UnityControl-fixture-tiling`.
The subsequent documentation-only checkout head is `f44d9bab82f694e684bd6b6a7bf51e771cea885d`;
runtime code/image remain the versions above. The original checkout, its HEAD and
21,344-byte dirty tracked patch remain unchanged; before/after SHA-256 is
`eb81fe233ebd1cb3afa59c8cccb051e1bed428e19296cb9fad9ca25ca4cc4976`.
The rollback image remains tagged `urmir-jazzy:before-fixture-20261102`.

The live container mounts `docker_fixture_calibration` at `/root/.config/ur16e`.
Fixture calibration remains deliberately disabled pending physical measurements;
the save service is available. No production Plan, Execute, robot motion or vacuum
command was sent. No headset connection or physical fixture execution was tested.
See `summary.json` and the sanitized manifests for exact identities and limits.

The deployment starts from the exact running application image, retains its ROS
Jazzy/MoveIt/MTC/endpoint dependencies, and freshly builds all ten local application
packages without network access. Generated fixture interfaces, the complete MTC
application and all mounting-model launch consumers must be installed together.
The existing 55.25 mm adapter correction gives 145 mm axial flange-to-TCP reach;
this deployment does not shorten the adapter a second time.

The previous MTC-only deployment helper is insufficient for this update. New
message/service Python types require a fresh ROS-TCP endpoint, while new model
defaults require regenerated descriptions for the driver, RSP, MoveGroup and MTC.
The same bringup arguments and relevant environment are retained, and a fresh
bringup generates new parameter files. Saved expanded descriptions from the old
process must not replace the corrected model.

The passive probe reports both a full root link/joint signature and a comparison
signature separating only joint `safety_controller` children. Before deployment,
RSP had driver-only soft-limit metadata on all six arm joints while MoveGroup/MTC
omitted it; all other link/joint structure agreed. This documented baseline
difference stays visible, with its numeric values, in the verification record.
Hard limits, origins, axes, visual/collision geometry and inertials remain part of
the comparison. This deployment does not change or consolidate safety settings.

The existing Compose project is `docker`, service `ur_container`, container
`urmir-jazzy`. The updated configuration adds the named `fixture_calibration`
volume at `/root/.config/ur16e` (normally `docker_fixture_calibration`). Preserve
any existing calibration file before recreation. Do not remove that volume.
The shipped fixture template stays disabled until measured calibration exists.

Before activation, retain the original image/tag, Compose file, source state,
container configuration and scoped private runtime backups on Nordbo. Verify the
production application is idle before stopping it. Rollback uses those captured
identities, configuration and image; it must not reset robot/IO state or submit a
motion request. The actual activation and rollback decision belong to the
operator, not this evidence recorder.

The expected manifests are `staging.json`, `build-result.json`,
`test-result.json`, `activation-result.json` and `final-verification.json`.
Staging records the original image, source archive hash and unchanged live state.
Build evidence identifies the actual `exitCode`, `image`, `packages` and disabled
network. Test evidence includes actual `reports`, `failures`, `unexpectedFailures`
and two isolated `replays`, including any independently known baseline assertion.
Test, activation and final manifests record an explicit `success` boolean. Final
evidence explicitly records `robot_motion_requested`, `execution_requests_sent`,
`io_commands_sent` and `production_plan_requests_sent` as false. Network-disabled
synthetic planning replay is recorded separately from production operations.

After collecting evidence, run:

```powershell
python Tools/RobotFrames/NordboFixtureDeployment_20261102/record_evidence.py --outcome verified_deployed
```

Use `--outcome staged`, `blocked` or `rolled_back` when appropriate. The recorder
only reads local artifacts and writes `summary.json`; it cannot deploy, build,
connect to ROS/SSH or signal processes. Keep private environment/parameter backups
on Nordbo and include only sanitized metadata and hashes here.
