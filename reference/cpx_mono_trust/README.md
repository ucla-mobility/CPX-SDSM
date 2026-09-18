# CPX-Mono trust packages (reference copies)

Unmodified copies of CPX-Mono's three trust-related ROS 2 packages
(`ros2/src/global_trust_perception`, `global_trust_tracker`,
`local_trust_estimation`), kept here so their code is browsable/editable
directly in this repo without needing CPX-Mono checked out alongside it.

**These are not part of the `ros2_ws` colcon build.** They live outside
`ros2_ws/src` on purpose, so `colcon build` never tries to compile them —
they depend on things this repo doesn't have (a real LiDAR/Autoware stack
for `local_trust_estimation`'s per-object sensor-confidence nodes,
`mmcooper_fuse`'s spatial WBF clustering, `lichtblick` visualization, CPX-Mono's
own `sdsm_interfaces` message set).

The package that actually runs in this repo is
[`ros2_ws/src/sdsm_trust_perception`](../../ros2_ws/src/sdsm_trust_perception),
whose `sdsm_trust_perception/global_trust_perception/` subpackage is a
pure-Python port of `global_trust_perception/global_trust_perception/pipeline`
and `trust_calculations` here, adapted to this repo's
`SensorDataSharingMessage` wire format — see that package's own docstrings
for exactly what was and wasn't carried over (notably: `mmcooper_fuse` and
`lichtblick` were not ported).

To refresh these copies after CPX-Mono changes, re-copy from
`../CPX-Mono/ros2/src/<package>` (adjust the path to wherever CPX-Mono is
checked out locally).

## What's in each folder

Each subfolder has its own `README.md` with a short "this repo" note at the
top (relationship to `sdsm_trust_perception`, what was/wasn't ported) —
`global_trust_perception`'s and `local_trust_estimation`'s notes sit above
their original CPX-Mono README content; `global_trust_tracker` had none, so
its note is the whole file:

- [`global_trust_perception/README.md`](global_trust_perception/README.md) — the Layer 2 (cross-agent) trust judge this repo's port came from.
- [`global_trust_tracker/README.md`](global_trust_tracker/README.md) — the standalone SORT tracker node; this repo has an inline equivalent already.
- [`local_trust_estimation/README.md`](local_trust_estimation/README.md) — the Layer 1 (local, sender-side) scoring package, not ported into this repo.
