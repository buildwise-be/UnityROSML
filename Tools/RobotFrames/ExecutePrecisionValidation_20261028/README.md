# MTC precision fix deployed on Nordbo — 2026-10-28

The valid 96-point plan reached Execute, but the deployed validator was still the
old October 23 baseline. Planned/current worlds differed only in one quaternion
coefficient by **6.94e-18**. Collision positions, shapes and octomap were identical.
The complete fresh scene replay rejects with the old header and passes with the
already reviewed precision correction.

After the owner's approval, “Deploy fix and restart MTC only”, the reviewed header
and regression fixture were deployed to the durable host checkout and container.
All twelve other runtime C++/header files match the previous container source.

- Exact-production-image package/application build passed; all **14 validator
  cases** and the full captured-scene replay passed. **9/10 CTest targets** passed;
  the sole failure is the unchanged documented scene-effects assertion.
- Only MTC restarted: old PID 632 became **1399**. Driver **420**, MoveIt **723**,
  endpoint **631** and every other recorded original process stayed unchanged.
- Exact argv/environment/cwd and nine parameter files were retained. Model/tool
  hashes and fresh six-joint feedback match. Supervisor **1387** follows original
  ROS launch **416**, signalling only its owned MTC child when that launch exits.
- The verified four-file COPY-only image layer is available as
  `urmir-jazzy:latest` and `urmir-jazzy:mtc-precision-20261028`, image
  `sha256:5cd92eb4c3e4a3ddcd6482c7656fd5349d277cd02db6185c1c0cb9632a242656`.
  The previous image remains `urmir-jazzy:before-mtc-precision-20261028`.
  The live container was never restarted or recreated; its writable install and
  the image used for future recreation both contain the corrected executable.

No Plan, Execute, robot motion or IO command was issued. No dependency rebuild,
registry push, commit or Git push was performed. Create a **fresh plan**, inspect
its preview, then use the separate Execute action when ready. This server fix
requires no Unity rebuild. Hardware execution itself remains untested by this work.

Evidence: [summary](summary.json),
[activation](NordboDeployment/activation-result.json),
[final provenance](NordboDeployment/final-verification.json),
[image result](NordboDeployment/image-result.json),
[CTest log](NordboDeployment/output/ctest.log) and
[startup log](NordboDeployment/runtime/mtc.log).

Scoped source/binary/parameter/environment backups and remote manifests are under
`/home/nordbo/.cache/codex/mtc-precision-20261028/` on Nordbo. Private environment
and raw parameter backups remain there. The runtime log in the live container is
`/tmp/codex-mtc-precision-20261028/mtc.log`. The Nordbo operational/API notes record
the deployment and the known baseline failure.
