# Global Trust Tracker (`global_trust_tracker`)

> **Note (this repo):** unmodified reference copy from CPX-Mono — not part of
> this repo's `ros2_ws` colcon build (see
> [`reference/cpx_mono_trust/README.md`](../README.md)).

A small standalone ROS 2 node (`tracker_node.py`) that runs one SORT
(Kalman constant-velocity) tracker per sender. It subscribes to raw SDSM
traffic on `/perception/global_trustworthiness/sdsm` and, for each incoming
message, publishes a `TrackUpdate` whose per-detection arrays are parallel
to that message's objects: a stable track id plus the tracker's own
position/velocity estimate for it.

`global_trust_perception` (the trust judge) consumes this `TrackUpdate`
stream for two things:
- the kinematic-consistency check, which predicts each track's position
  from its previous Kalman state and flags implausible jumps;
- the stable track id that keys the deferred grace ledger for
  uncorroborated ("other_only") detections.

In CPX-Mono's launch order this node must start **before** the trust-judge
agent nodes, since they depend on its `TrackUpdate` stream from the first
frame.

In this repo, the equivalent SORT tracker is not a separate node — it lives
inline inside
[`sdsm_trust_perception/global_trust_perception/tracking/`](../../../ros2_ws/src/sdsm_trust_perception/sdsm_trust_perception/global_trust_perception/tracking),
with each judge running its own tracker directly rather than subscribing to
a shared tracker node. This folder is here for reference/comparison only.
