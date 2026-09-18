# Trustworthy Cooperative Perception (`global_trust_perception`)

> **Note (this repo):** unmodified reference copy from CPX-Mono — not part of
> this repo's `ros2_ws` colcon build (see
> [`reference/cpx_mono_trust/README.md`](../README.md)). This is the package
> [`ros2_ws/src/sdsm_trust_perception`](../../../ros2_ws/src/sdsm_trust_perception)
> was ported from: its `pipeline/` and `trust_calculations/` are the same
> Layer 2 cross-agent judge (kinematic checks, corroboration, deferred
> ledger, reputation, dynamic gate), adapted to this repo's
> `SensorDataSharingMessage` wire format. `mmcooper_fuse/` (N-way spatial
> clustering) and `lichtblick/` (Foxglove visualization) were **not**
> ported — see the running package's own docstrings for exactly what
> changed. Everything below this line is CPX-Mono's original documentation
> and describes CPX-Mono's own 5-sensor intersection demo, not this repo.

## Methodology

In pipeline order:

1. **Ego-centric decentralized trust.** Every agent independently scores every other agent from its own point of view. There is no central authority; each node owns one `TrustEngine` and one reputation database.
2. **Information-hiding message codec** (`perception_message.py`). The SDSM wire format is known to exactly one module; everything else consumes plain Python tuples. Swapping the message type means editing one file.
3. **Frame bucketing.** Incoming messages are buffered and processed in fixed windows aligned to the publish rate, so all agents' reports for the "same moment" are compared together.
4. **SORT multi-object tracking** (`global_trust_tracker/tracker_node.py`). A separate ROS2 node subscribes to the SDSM topic, runs a Kalman constant-velocity tracker (state `[x, y, w, l, vx, vy]`) per sender, and publishes one `TrackUpdate` message per SDSM on `/perception/global_trustworthiness/tracks`. The trust pipeline consumes the **stable track id** (which keys the deferred ledger) and the Kalman state (which feeds KDS). The tracker also publishes a `confirmed` flag (set after `min_hits` consecutive detections), but **the trust pipeline does not read it** — see item 6a. The tracker must be started before the agent nodes (see Running it).
5. **N-way cross-agent fusion (MS-PSF phase 1)** (`mmcooper_fuse/adapter.py` + `mmcooper_fuse/fusion.py`). Every agent's detections (ego included) go into ONE **reputation-weighted** Multi-Source Promote-Suppress Fusion over rotated bird's-eye-view boxes (ego reliability pinned to 1.0, peers weighted by their current reputation, per-object `obj_local_scores` as the detection confidences). The fusion groups overlapping detections into one **cluster** per real-world object (`{agent_id: detection_idx}`, at most one per agent); each agent's `matched` / `ego_only` / `other_only` buckets are derived from the shared clusters. This is the *judging* fusion — it runs over **everyone, ungated**, because an agent cannot be scored against a consensus it was excluded from forming. What that cluster membership is *worth* as corroboration is trust policy, not geometry, so it is computed by `corroboration_support` in `trust_calculations/consistency.py`: **reputation-mass support** = Σ (reputation × local certainty) of the *other* peers in a detection's cluster (ego excluded); an `other_only` detection whose support does not clear `SUPPORT_THRESHOLD_THETA` is `uncorroborated`. Support being reputation *mass*, not a head count, is what stops k low-reputation colluders from corroborating each other's ghost; weighting each peer's contribution by the certainty it reported means a hesitant witness corroborates less than a confident one.
6. **Weighted ego-consistency scoring** (`consistency.py`). The buckets map to weighted Correct / Incorrect / Held (see the table below). Correct earns the reporter's certainty `c`. `ego_only` items (objects ego sees that the other agent missed) are always Held — agents are not penalised for failing to see what ego sees. The rules are fully asymmetric: only an agent seeing *more* than ego can be penalised (via the deferred ledger for `other_only` items).
6a. **Deferred verdicts for uncorroborated ghosts** (`deferred.py`). An `other_only` detection can be a fabricated ghost or a real object occluded from every peer — the local score can't tell them apart (high confidence is the signature of both). Time can, so judgment is *delayed*, keyed by `(agent, track_id)`: held during a grace window of `T_DEADLINE_S` seconds (occlusion assumed), then **back-paid in full as Correct** if a peer corroborates it in time, or **back-charged as Incorrect** once the deadline passes (and charged every frame after). The grace window is measured from the detection's **first sighting**: a sender's claim that an object *exists* is taken at face value the moment it makes it. There is deliberately **no tracker-confirmation gate** here — the local pipeline has already dropped what its sender was unsure of, so re-filtering on `min_hits` would double-filter the same question and delay both the charge and the credit by the warm-up. A track that dies **inside** the grace window is forgiven (dying does not make an uncorroborated report a fabrication, and a real tracker re-IDs ~10–14 % of ids frame to frame, which would drain an honest sender); one that outlives the deadline is back-charged on death as well as on expiry, closing the spawn-and-abandon cycling hole. Each held frame settles exactly once.
7. **Kinematic-Dynamic Consistency Score, KDS** (`consistency_checks/kinematic_checks.py`), approximating CooperFuse eq 2.2-2.8. A continuous score in `(0, 1]`: a minimum-jerk trajectory fit from each track's previous Kalman state to its new reported position, times a Dubins-path-length ratio (straight-line distance over the shortest bounded-turning-radius path connecting the previous and reported headings) approximating Reed-Shepp's curvature constraint — Dubins is Reed-Shepp's forward-only special case; reversal maneuvers are a disclosed gap, not an oversight (see the module's README). The curvature term is **soft-gated** by per-detection `P(vehicle)` (from `c_i`, the same class-probability vector MS-PSF fuses via eq 3.19) rather than a hard class switch: `KDS_curvature_eff = 1 - P(vehicle)·(1 - KDS_curvature_raw)`, so an uncertain or misclassified pedestrian/cyclist detection doesn't take the full car-shaped penalty. Computed *before* MS-PSF runs (item 5) since it feeds fusion: it reweights the fused box's **orientation only** (KDS-weighted circular mean, itself softly class-gated the same way; position/scale are unaffected), and separately discounts how much a matched pair counts as corroborating evidence in item 6 below (`matched_certainties × KDS`) — the only place it touches reputation. See `consistency_checks/README.md` for what's ported from the paper versus approximated.
8. **Size-consistency score, SS** (`consistency_checks/attribute_checks.py`), approximating CooperFuse eq 2.5. A continuous width/length agreement ratio for matched pairs; credited only once the other agent's reputation exceeds 0.70 (gate applied at the call site now, not inside the check). Diagnostic-only, same as its retired boolean predecessor — does not feed reputation.
9. **Additive reputation update with baseline decay** (`reputation.py`). `S_frame = (C − I) / N_total`, `R_new = clip(R_old + 0.10 × S_frame, 0, 1)`. When nothing was scoreable (`N_total = 0`) the reputation decays toward the 0.5 default instead of freezing.
10. **Dynamic trust threshold gate with kinematic freshness** (`reputation.py`, `reputation_multipliers/kinematic_freshness.py`, `reputation_multipliers/sender_motion.py`). The gate is checked **first** each frame against the *effective* reputation `R_eff = R_old × F × V` (V is the persistence penalty, next item): `gate_passed = R_eff >= tau(R_eff)`. `F ∈ [0.5, 1]` bounds how far the sender's own tracked speed could have carried it during THIS message's own send-to-receive latency: `blind_distance = speed × latency`, where `speed` comes from `SenderMotionHistory` (a small dedicated Kalman filter over each sender's `ref_pos_x/y` across messages, independent of the SORT object tracker) and `latency` from `perception_message.send_latency_of` (the message's own send stamp vs. ego's receive time — trusted input, not itself verified here). Deliberately based on THIS message's latency rather than the gap since the sender's *previous* message: a sender silent for minutes that then sends a message with negligible latency reads as fully fresh, since a long silence gap doesn't make the new snapshot any less current. `F` is fully fresh (`=1`) within a 1 m grace band (negligible blind distance — includes any latency when `speed ≈ 0`), then decays such that a sufficiently large blind distance fails the gate **regardless of reputation** (the floor is derived from the gate's fixed point, so this guarantee survives retuning — see the two-regime dichotomy in `kinematic_freshness.py`). Timestamps are assumed honest — a dishonest send-stamp is a different trust concern, not this factor's job; catching implausible reported *motion* is instead the kinematic consistency check below. F is transient: it never touches stored reputation or the DB; a blind-distant message costs that frame's admission only. A gate-failing agent is still fully scored (so its reputation can recover), but its objects are withheld from all downstream use this frame — excluded from the fused output and drawn bright red in the visualization.
11. **Persistence penalty against on-off attackers** (`reputation_multipliers/persistence_penalty.py`). The `V ∈ [0.4, 1]` in the gate formula above: each drop in an agent's raw reputation feeds a per-agent leaky risk accumulator whose memory is tuned as a real-time half-life (30 s at the flush rate; the decay coefficient is derived from it, like the freshness base). An agent that alternates good and bad frames — banking reputation between hits so raw `R` never stays low — accumulates "drop debt" and is withheld even on its well-behaved frames; the penalty strength is anchored to an intolerable duty cycle (bad 30 % of frames at full severity converges to `V = 0.5`), and a second short-window channel adds a prompt sting on the drop frame itself. Gate-only like F: stored reputation and the DB never see V — the memory lives in the tracker's risk state, which also avoids a drops-amplify-drops feedback loop — and once the attack stops the risk decays and the agent earns back admission (floored at 0.4, never fatal). Reappearing after an absence decays the banked risk for the missed time, and the reputation difference across the gap is not scored as a drop (absence decay already charged it). Interactive design doc: `visualizations/persistence_penalty.html`.
12. **Persistent reputation with session history and absence decay** (`persistent_reputation_tracker.py`, `reputation_multipliers/absence_decay.py`). SQLite (WAL mode), one session per continuous interaction, batched writes every `BATCH_SIZE` frames, and reputation *seeding with absence decay*: a re-encountered agent starts from its last stored reputation decayed toward the 0.5 baseline by the wall-clock time it was away (half-life of the above-baseline deviation ≈ 4.6 days; a month of absence is effectively a reset to 0.5). Downward-only: a below-baseline reputation does not drift back up, so a burned agent cannot launder its score by disappearing. Agents whose only history predates the `ts` column reseed at the 0.5 default (age unknowable).
13. **Trust-weighted score service.** `GetTrustScore` returns `R(agent) × local_score`, letting downstream consumers discount another agent's data by its earned reputation.
14. **Output admission tiers.** After scoring, each gate-passing agent's detections are admitted to the OUTPUT by a per-object tier policy (`FrameStats.admitted`): an agent at or above `HIGH_TRUST` (0.95) passes **everything immediately** (optimistic trust — a proven sensor's late-breaking object acts without waiting for a second witness); an agent in `[tau, HIGH_TRUST)` passes only its **corroborated** detections, muting uncorroborated ones from the output while still grading them (so a fresh R=0.5 Sybil's ghosts never reach the vehicle's controls). Gate-failing agents pass nothing. Grading and admission are decoupled: a muted-but-truthful agent still earns its way up.
15. **MS-PSF phase-2 display fusion** (`mmcooper_fuse/adapter.py` + `visualization.py`). A second MS-PSF fusion over the *admitted* set only (weighted by the just-updated reputations) produces the fused boxes shown in the live 3D view. This is display-only here and runs outside the flush timing; the production pipeline forwards the admitted detections rather than re-fusing. The right panel annotates each fused box with its class, fused certainty, and contributor list (ego first); muted/held objects appear as dashed grey outlines.
16. **Trust-output rebroadcast** (`agent.py`, `perception_message.py`). After `process_frame` returns, ego rebroadcasts the original SDSM of every OTHER agent whose `FrameStats.trusted` (Stage-0 gate) passed this frame, on a per-ego topic `pmsg.trust_output_topic(agent_id)` (`/perception/global_trustworthiness/trust_output/agent_<id>`). The rebroadcasting ego's identity is carried by the topic **name**, not the payload, so the SDSM stays standard J3224 — a consumer learns whose trust view a rebroadcast represents from the channel it arrived on, and can subscribe to one ego's view or discover all of them. The rebroadcast uses a **different message type from the input**: `SdsmTrustOutput` (`pmsg.OutputMessage`), not `SdsmPayload`. `pmsg.with_global_score` carries the entire object payload forward field-for-field and swaps only the trust annotation — the input's per-object `obj_local_scores[32]` is **dropped** (it is the sender's own view of its detections, which has no meaning once ego has judged them, and it costs 128 bytes on an 802.11p link) and replaced by two singular floats: `global_score`, this ego's `R_new` for that sender, and the sender's own frame-level `local_score`, passed through unchanged. Both are kept rather than one pre-multiplied product, which a consumer could not decompose again. Decode with `global_score_of`. The value is a per-ego reputation, not yet a cross-ego consensus (see Known limitation). **Only gate-passing senders are ever published here** — the trusted set is expressed by *presence on the topic*, never by a flag, so a withheld sender's perception simply does not exist downstream and no consumer can act on it by forgetting to check a boolean.
17. **Diagnostic verdict channel** (`trust_verdicts.py`, `TrustVerdicts.msg`). Vehicle-internal and visualization-only, published per-ego on `/perception/global_trustworthiness/verdicts/agent_<id>` — one message per judged sender per flush, correlated back to the judged SDSM by `msg_cnt`, the same convention `TrackUpdate` uses. Unlike the output topic above it carries **every** sender, gate-passing or not, plus a per-detection verdict label — `matched` (corroborated by ego or by peers), `deferred` (held inside the ledger's grace window), `uncorroborated` (the window closed and it is being charged every frame) — a dense per-detection `admitted` flag, and the ego detection indices this sender corroborated (so a viewer can colour ego's own boxes without re-deriving cluster membership). It exists precisely because the filtered output *cannot* represent a rejected sender and a visualization has to. **Nothing in the production path may consume it:** acting on a sender found here with `trusted=false` would defeat the gate it just failed. The engine emits these labels as strings (`trustworthy_perception.VERDICT_*`), which keeps it ROS-free; the mapping onto compact wire integers lives only in the codec, so a visualization process never imports the trust pipeline to colour a box.

18. **Diagnostic fused-scene channel** (`fused_objects.py`, `FusedObjects.msg`). Vehicle-internal and visualization-only, published per-ego on `/perception/global_trustworthiness/viz/fused_boxes/agent_<id>` — one message per flush, carrying the consensus boxes MS-PSF produced this frame so a viewer can draw the fused estimate beside the inputs that made it. Fusion is per-ego (it combines that ego's own detections with the peers it heard), so the ego's identity rides in the topic **name**, matching the two channels above. **It costs the pipeline nothing:** `fuse()` already derives this geometry on every frame and discards it — the trust decision reads `clusters` and nothing else (see `mmcooper_fuse/adapter.py`'s display-derivation note) — so the engine only holds a reference to what it already built, exposed as the read-only `TrustEngine.last_fusion`, and `agent.py` skips packing the message entirely unless something is subscribed. `last_fusion` is reset at the top of every `process_frame`, so a late reader gets `None` rather than a previous frame's boxes. **One exception to "costs nothing":** a frame where no peer was heard returns before the fusion stage, which would publish an empty scene and make the viewer clear the whole layer — a blink, since unsynchronized senders miss buckets routinely. Setting `TrustEngine.fuse_solo_frames` makes such a frame fuse ego's stream alone, so the viewer gets a *current* answer rather than an empty one or a stale held-over one. That is a real extra `fuse()` call, which is why it is opt-in: `agent.py` sets it only while something is subscribed, and a headless run pays nothing. It sits inside that peerless branch, which returns before the gate, the ledger and the reputation update run at all, so no trust state is touched either way. It deliberately does **not** carry contributor identities: which input fed which fused box is left to the eye, which keeps the message flat and the fused layer independent of how the inputs are drawn. **Nothing in the production path may consume it** — a fused box has no sender and no reputation.

### Per-frame execution order

The list above groups by concern; this is the exact order `TrustEngine.process_frame` runs each flush, for each other agent:

1. **Read stored reputation `R_old`.** In-memory dict; on first contact with an agent, seeded from its last SQLite record with **absence decay** applied to the wall-clock gap since that record (or `REPUTATION_DEFAULT` for strangers and pre-`ts` legacy history).
2. **Trust gate check (Stage 0).** `R_eff = R_old × F × V` (F = kinematic freshness from the sender's tracked speed times this message's own transmission latency, 1.0 when no sender position data — or when the latency measurement itself is unusable, i.e. negative past `CLOCK_SKEW_TOLERANCE_S`, which is logged as a WARNING and treated as "no usable freshness evidence" rather than as a verdict about the sender; V = persistence penalty from this agent's drop history, 1.0 while its reputation is not falling); `gate_passed = R_eff >= tau(R_eff)`, logged immediately. This decides whether the agent's *data* is admitted downstream this frame. The rest of the pipeline runs either way — a failing agent keeps being scored so it can earn its way back. Note: a brand-new agent starts at R = 0.5 and `tau(0.5) > 0.5`, so new agents fail the gate until their reputation climbs over a few clean frames.
3. **Tracking.** Track ids and Kalman state arrive as `TrackData` forwarded from the tracker node's `TrackUpdate` messages (buffered in `agent.py` and passed into `process_frame`). Per detection: stable track ID, Kalman position and velocity estimate. If no `TrackData` arrives (tracker node not running), `other_only` detections are simply held — without a stable track id the ledger cannot grade them over time.
4. **Kinematic consistency check.** Uses the tracking outputs: predicts each track's position from the previous frame's Kalman state and flags detections that jumped implausibly.
5. **N-way MS-PSF phase-1 fusion.** One reputation-weighted MS-PSF pass across ego + **all** agents (runs once per frame, before the per-agent loop, ungated), producing clusters `{agent_id: detection_idx}` per real-world object. `corroboration_support` then values that membership as per-detection **reputation-mass support** (Σ reputation × certainty over the other peers in the cluster). Each agent's `matched` / `ego_only` / `other_only` buckets are derived from these shared clusters — all agents matched against one consistent world model, not independent pairwise matchings.
6. **Kinematic demotion + attribute agreement + corroboration.** Flagged matched pairs are demoted (matched → ego_only); surviving pairs get the size-agreement check (logged only). Then support scoring: an `other_only` detection whose peer support does not clear `SUPPORT_THRESHOLD_THETA` is counted `uncorroborated` (`unc=` in the logs) and fed to the deferred ledger.
7. **Weighted consistency + deferred ledger → C / I / held.** `matched` → Correct weighted by the reporter's certainty; `ego_only` → always Held (agents are not penalised for what ego sees); `other_only` → the ledger, which holds it during the grace window and settles it (back-pay Correct on corroboration, back-charge Incorrect on expiry/death). The ledger's settled deltas add into C and I here.
8. **Reputation update.** `S_frame = (C − I) / N_total` (now weighted floats), `R_new = clip(R_old + 0.10 × S_frame, 0, 1)`; decays toward 0.5 if nothing was scoreable. `R_new` becomes the input to *next* frame's gate check.
9. **Admission tiers (Stage 5).** The Stage-0 verdict is reported in `FrameStats.trusted`; the per-object output tier is reported in `FrameStats.tier` / `FrameStats.admitted`: reject (gate failed), corroborated-only (`[tau, HIGH_TRUST)`), or everything (`≥ HIGH_TRUST`). agent.py fuses/forwards the admitted set; gate-failing agents are drawn bright red on the raw panel and muted objects as dashed grey.
10. **Store locally.** `R_new` goes to the in-memory dict immediately and to SQLite via `record()`, batch-flushed every `BATCH_SIZE` frames.

**Untrusted agents cannot influence anyone else's trust score.** Trust scoring is strictly pairwise: ego's score for agent X is computed *only* from ego's own detections vs X's detections — no third agent enters that calculation anywhere. So a gate-failing agent already has zero effect on other agents' reputations by construction. This matters for the future N-way consensus path (see Known limitation): when other agents start acting as *verifiers* for each other's detections, the verifier set must be filtered to gate-passing agents only, and the gate flag in `FrameStats` is the signal to use for that.

## Running it

All commands run from the **colcon workspace root** — the `ros2_ws` directory that
contains `install/` (NOT `ros2_ws/src/...`). For example:

```bash
cd /workspaces/mobilityLab/ros2_ws     # adjust to wherever your ros2_ws is
```

### 0. Prerequisites (first time only)

Source the ROS environment, and install the Python deps that have no apt
package (`filterpy`). On a fresh container `pip` itself may be missing:

```bash
source /opt/ros/kilted/setup.bash          # adjust distro: ls /opt/ros/
apt update && apt install -y python3-pip   # if 'pip3: command not found'
python3 -m pip install -r src/CPX-Mono/ros2/src/global_trust_perception/requirements.txt --break-system-packages
```

`--break-system-packages` is required on Ubuntu 24.04 (PEP 668). `numpy` and
`scipy` usually come with ROS already; in practice only `filterpy` and
`ensemble-boxes` install from scratch.

To install a single package (e.g. while iterating):

```bash
python3 -m pip install ensemble-boxes --break-system-packages
```

**Adding a new dependency:** add the package name to `requirements.txt` and re-run the install command above — it installs everything in the file at once. No pip entry is needed for Python standard library modules (`sqlite3`, `json`, `math`, etc.) — those are always available.

### 1. Build & source

```bash
colcon build --packages-select global_trust_tracker global_trust_perception
source install/setup.bash      # zsh users: source install/setup.zsh
```

### 2. Launch everything (one command)

The whole intersection scene — the tracker, the five sensor agents, and the scene visualizer — starts from a single launch file:

```bash
source install/setup.bash
ros2 launch global_trust_perception intersection_sim.launch.py
```

The tracker is first in the launch description because the agents consume its TrackUpdate stream. The launch file keeps visualization off so the five nodes don't each pop a window; to watch one agent's live 3D plot, run that agent manually with `-p visualize:=true` (below).

**Manual alternative (one terminal per node).** Start the tracker first, then any subset of agents. Each agent needs a unique `agent_id` (1-4 are the cars, 5 is the RSU) and node name:

```bash
source install/setup.bash
ros2 run global_trust_tracker tracker                                               # terminal 1
ros2 run global_trust_perception agent --ros-args -p agent_id:=1 -r __node:=agent_1   # terminal 2
ros2 run global_trust_perception agent --ros-args -p agent_id:=5 -r __node:=agent_5   # the RSU
```

Add `--ros-args -p visualize:=true` to an agent to open a live 3D plot window that updates each flush. Both panels share the same axis scale and color scheme: each unique **combination** of agent IDs that co-detected an object (e.g. `{ego, 1}`, `{1, 2}`) gets a distinct color from a 12-color palette, with a legend entry per combo. Raw boxes on the left are drawn with correct orientation from J2735 heading; agent IDs are annotated above any box seen by 2+ agents. Rejected agents are drawn **bright red**. The right panel shows the phase-2 fused boxes in the same combo color, with the fused certainty in bold; muted/held detections are dashed grey. Requires a display (not headless).

**Replay mode.** `agent.py` also takes a `mode` param, `sim` (default) or `replay`. In `replay` mode the agent publishes no synthetic traffic of its own — it only subscribes to whatever already publishes prerecorded `SdsmPayload` messages on the perception topic (e.g. `ros2 bag play`):

```bash
ros2 run global_trust_perception agent --ros-args -p agent_id:=1 -p mode:=replay -r __node:=agent_1
```

Send latency is measured the same way in replay as in sim: `pmsg.send_latency_of` against the sender's own embedded stamp. What is replayed here is the *sensor* data, not the SDSMs — `sdsm_publisher` re-encodes it live and stamps live wall-clock (it only freezes the stamp under `offline=True`, which only `evaluate_offline` sets), so the two clocks are directly comparable and the measured latency is a real transmission delay.

**Frame window.** `flush_frame` scores together every message whose sensor frame was **captured** inside one `flush_interval_s` of scene time — grouped by capture, not by arrival. It defaults per mode — 0.5 s (2 Hz) in `sim`, 50 ms (20 Hz) in `replay` — and is overridable:

```bash
ros2 run global_trust_perception agent --ros-args -p mode:=replay -p flush_interval_s:=0.02
```

The two modes differ because sim publishes its own traffic on `sim_world`'s clock: that scene advances a fixed distance *per tick* and reports speeds as metres per `sim_world.TICK_DT_S`, so the sim publish timer is tied to that constant and its window has to match, or windows would sit empty. Replay consumes whatever a bag emits and is free to run at the production target. Overriding sim to a shorter window is allowed but leaves publishing at 2 Hz, so most windows will be empty.

**Capture time, not arrival — and why they are two knobs.** Agents watching one instant do not finish describing it together: each runs its own perception chain first, and a denser cloud takes longer (on the `scene_01` pair, ~33 ms for the infrastructure against ~92 ms for the vehicle). Grouped by arrival, that spread decided which frame a report landed in — those two fell in the same 50 ms window only ~41% of the time, and in the rest ego contributed nothing, so every peer detection was `other_only` *by construction* and the frame produced zero matches however well the boxes overlapped.

Widening the window fixed the pairing and created a worse problem, because the window is also the span of real time a frame claims was **simultaneous**. At 150 ms and 30 m/s it pairs boxes up to 4.5 m apart — roughly the entire IoU association budget for a car (`mspsf_tau_graph = 0.05`) — so beyond a certain speed the widening that fixed pairing is what breaks association, and an honest peer is charged as a fabricator for having a slower chain.

Each sender therefore reports its own capture lag (`obj_measurement_time_ms`, see `local_trust_estimation`'s `capture_lag_ms`) and the receiver recovers the capture instant as `arrival − lag`. That splits one overloaded knob into two with one reason to change each:

| Parameter | States | Sized from |
|---|---|---|
| `flush_interval_s` | how wide an *instant* is | bounded **below** by the streams' sensor offset, above by how much real motion may be called simultaneous |
| `close_budget_s` | how long a bucket waits past its own end for slower senders | the slowest perception chain in the graph |

**`flush_interval_s` has a floor — do not minimise it.** Capture grouping removes the *processing* spread; it does not remove the *sensor* offset, because the agents' sensors are not synchronised (on the `scene_01` pair the residual stream alignment is ~30 ms). Bucketing is a fixed grid, so a pair captured `d` apart is split whenever a boundary falls between them — probability ≈ `d / flush_interval_s`. Measured on that pair:

| `flush_interval_s` | split | co-occurrence |
|---|---|---|
| 0.05 | ~60% | ~40% — reputation oscillates between its bounds |
| 0.10 | ~30% | ~70% |
| 0.15 | ~20% | ~80% |

Below the sensor offset you recreate the original problem on capture time instead of arrival time. The gain from capture grouping is not a smaller window — it is that the window now bounds a span of *capture instants* rather than of arrivals, so the same width carries a much stronger claim.

A message arriving after its bucket was judged is **dropped and counted**, not folded into the next one — that would file a report about the past under a present frame, and let arrival order change a verdict. Watch for `[DIAG] dropped N message(s) for already-judged bucket`; it means `close_budget_s` is under the real chain spread. A sender that leaves the lag field at zero groups by arrival exactly as before, so this is backwards compatible.

The two latency legs stay on deliberately different clocks: capture lag is **scene** time (reproducible, so grouping cannot depend on playback rate), while send latency is **wall** time (a transmission delay is a physical fact playback does not change) and remains the freshness gate's input. They are never added into one number.

The window also sets the engine's decay cadence: the persistence penalty decays once per flush, so `TrustEngine` is constructed with `flush_hz` and sizes its half-life from it. Changing the window without telling the engine would scale `V_HALF_LIFE_S` by the same ratio in wall-clock terms.

**At most one message per sender per window.** A sender that publishes faster than the flush timer, or whose message jitters across a window boundary, lands twice in one window; only the latest is scored (`agent._latest_per_sender`), since the older one is stale state the newer replaces. The `_diag_superseded` counter reports how often this happens — a nonzero rate means senders are drifting relative to the flush timer.

### The intersection scenario

The scene (defined entirely in `sim_world.py`) is an unmarked intersection with **five sensors** and **one pedestrian**:

| Sensor | Start (m) | Heading | Notes |
|---|---|---|---|
| Car 1 | (0, 6) | south (−y) | also reports a **persistent phantom** at (1, −1) nobody else sees |
| Car 2 | (1, −6) | north (+y) | |
| Car 3 | (1, −9) | north (+y) | |
| Car 4 | (−6, −1) | east (+x) | |
| RSU (agent 5) | (0, 1) | static, 360° | infrastructure sensor; drawn as a small box |

The **VRU** (pedestrian) stands at (2, −1.5). Every car moves 0.1 m per tick in a straight line; each sensor reports whichever *other* road users fall within 4.5 m **and** its forward/backward FOV cone (±60°). The RSU is omnidirectional. Cars start spaced out along their own lane (not just at the intersection) so most pairs begin out of range/FOV and reveal each other as they close in, instead of everything already being visible at tick 0; at the current tick rate (`sim_world.TICK_DT_S`, kept in sync with `agent._TICK_INTERVAL_S`) the full reveal sequence — every pair, plus the VRU to every car that ever sees it — completes in **~60 seconds**.

Two behaviours are worth watching:

- **The VRU is occluded from the cars at first** — only the RSU sees it, from tick 0. A car gains line of sight once it reaches the corner mark at (1, −1): Cars 2 (~38s), 4 (~47s), and 3 (~60s) pass through it and start reporting the VRU; Car 1 (driving straight down x = 0) never does. Once two sensors report the VRU together it becomes corroborated — **but not always in time**: the RSU's VRU sighting enters the deferred ledger's grace window at tick 0, which expires (`T_DEADLINE_S`) well before the earliest car reaches the mark at this spacing. The RSU's honest, uniquely-early observation is currently back-charged as if it were a fabrication before real corroboration arrives — the exact "good sensor in a bad spot" failure mode the ledger is meant to protect against, exposed by how far out these cars now start. Closing this gap needs either a much larger `T_DEADLINE_S` (tens of seconds) or pulling the mark closer to the RSU so a car reaches it sooner.
- **Car 1 is the scenario's liar.** Its phantom at (1, −1) is an `other_only` track no peer ever corroborates (`unc=1`), so the deferred ledger holds it through the grace window (`pend=1`) and then back-charges it (`+I` rises, Car 1's reputation falls) — a persistent fabrication punished over time.

Every tick — including tick 0 — is derived from the same range + FOV + occlusion rules above; there is no scripted seed anymore now that the cars start apart.

### Inspecting the scenario independently (no ROS)

`sim_world.py` has zero ROS dependencies (pure `math` + `typing`), so the world geometry — positions, range, FOV cones, occlusion — can be inspected without building, sourcing, or even Docker, using any Python 3 that can import the package:

```bash
cd ros2/src/global_trust_perception    # from the CPX-Mono repo root
python3 -c "
from global_trust_perception import sim_world as w
for sensor in (1, 2, 3, 4, 5):
    xy, dets = w.observe(sensor, tick=0)
    print(sensor, xy, [(d.object_id, round(d.x,2), round(d.y,2)) for d in dets])
"
```

`w.observe(sensor_id, tick)` returns `(sensor_xy, detections)` exactly as `perception_message.build()` consumes it — useful for checking a visibility change (new range, a moved car, a different FOV angle) before wiring it through the full pipeline. `w._visible(sensor_id, target_id, tick)` and `w.pos_at(entity_id, tick)` are handy for one-off checks (e.g. "at what tick does Car 2 first see the VRU?").

To also exercise the wire encoding (`perception_message.build()`, which packs these into `SdsmPayload` units), the ROS environment must be sourced first (it imports `sdsm_interfaces`), so that step needs the `ros2_dev` container:

```bash
source /opt/ros/kilted/setup.bash && source install/setup.bash
PYTHONPATH=src/CPX-Mono/ros2/src/global_trust_perception:$PYTHONPATH python3 -c "
from global_trust_perception import perception_message as pmsg
msg = pmsg.build(agent_id=5, counter=0)
print(msg.ref_pos_x, msg.ref_pos_y, msg.num_objects)
"
```

Neither path starts any ROS node, publishes to a topic, or touches the reputation DB — it's read-only scenario inspection, not a substitute for running the pipeline.

### Lichtblick / Foxglove 3D visualization

`scene_node` is a read-only visualizer that converts the SDSM parallel-arrays into `foxglove_msgs/SceneUpdate` so the Foxglove 3D panel can draw them as glTF cars (box fallback for non-vehicle `obj_type`). It never touches the trust pipeline.

```bash
source install/setup.bash
ros2 run global_trust_perception scene
```

It publishes three scenes; toggle the first two in one 3D panel (before vs after fusion), and keep the ground on:

- **`/perception/global_trustworthiness/viz/scene_raw`** — every sender's raw broadcast, objects tinted by the sender's own per-object local score (low-trust senders included).
- **`/perception/global_trustworthiness/viz/scene_trust`** — only gate-passing senders from the **selected ego's** perspective, tinted by `global_score`; held (gate-failed) senders are simply absent.
- **`/perception/global_trustworthiness/viz/scene_ground`** — a static dark asphalt plane with grid lines for a street-like ground reference (toggle with the `show_ground` param).

The `ego_source_id` parameter (default 1) picks which broadcaster is drawn as the sports car and, for `scene_trust`, whose per-ego `trust_output` topic to read. It is live-settable (e.g. from Lichtblick's Parameters panel), so you can flip between agents' views without restarting. Per-ego `trust_output` topics are discovered dynamically, so it works with any number of agents. Detected objects are scaled relative to their SDSM length (`_REF_CAR_LEN_M`), so bigger objects render bigger.

`SdsmPayload.msg` carries a heading for each *detected object* but not for the sender itself, so a sender's own car icon can't just decode a transmitted heading. Instead `scene_node` derives it from consecutive `ref_pos` updates (`atan2` of the position delta) and latches the last heading whenever a sender's position doesn't change between messages — so a car sitting still (e.g. stopped at a light) keeps facing its lane instead of snapping to a default orientation.

**Node parameters** (all live-settable from Lichtblick's Parameters panel):

| Parameter | Default | Purpose |
|---|---|---|
| `ego_source_id` | `1` | Which broadcaster is the ego (sports car) and whose trust view `scene_trust` shows |
| `car_scale` / `sports_car_scale` | `1.0` | Calibrate mesh size for a ~4.5 m car / the ego mesh |
| `model_roll_deg` / `model_pitch_deg` | `0` | Fixed glTF base orientation; Foxglove already handles Y-up, so roll/pitch stay 0 unless a mesh imports tilted |
| `model_yaw_deg` | `90` | Points the mesh nose along heading; the shipped car/sports meshes are authored nose-along +Y, so 90 rotates them onto the +X heading axis |
| `center_x` / `center_y` | `0` | Optional world-frame view offset |
| `show_ground` | `true` | Publish the asphalt + grid ground plane |
| `show_labels` | `true` | White billboard text over every entity (ego, senders, each detected object). Set `false` to leave only the coloured geometry — with many objects the labels otherwise stack into a wall of text |
| `ground_extent_m` / `ground_step_m` | `400` / `5` | Ground size and grid spacing, metres |
| `render_hz` | `4.0` | Scene republish rate (meshes are embedded each frame) |

**Set-up in Lichtblick / Foxglove:**

1. Start `foxglove_bridge` (`ros2 launch foxglove_bridge foxglove_bridge_launch.xml`) from a shell with the workspace sourced, and connect Lichtblick to `ws://localhost:8765` (Foxglove WebSocket).
2. Add a **3D** panel.
3. In its **Settings**: set **Frame → Display frame** to `map`, and under **Topics** enable `.../viz/scene_raw`, `.../viz/scene_trust`, and `.../viz/scene_ground`.
4. **Ground/grid** comes from `scene_ground` (the asphalt + grid the node publishes); no Grid layer needed. Tune it with `ground_extent_m` / `ground_step_m`, or turn it off with `show_ground:=false`.
5. The ego sits at `ref_pos` (origin by default); detected objects are at their real offsets (tens of metres out). Pan (right-drag / two-finger) toward them, then scroll to zoom.
6. **Toggle `scene_raw` vs `scene_trust`** visibility = before vs after fusion.
7. Add a **Parameters** panel to change `ego_source_id` and tune the orientation/scale knobs live.

### Per-object trust view (`trust_view`)

The second read-only visualizer, and the one to use when the question is *"what did the pipeline decide about **this object**?"* rather than *"what does the scene look like?"*. It publishes `visualization_msgs/MarkerArray` rather than `SceneUpdate`, so it drops into the same Lichtblick 3D panel as a point cloud or the local-trust pipeline's own markers.

```bash
ros2 run global_trust_perception trust_view --ros-args -p ego_source_id:=1
```

Three topics, one per layer, meant to be stacked in one 3D panel. Layer = topic on purpose: the panel's per-topic visibility is then the only enable/disable control needed, and hiding a layer also stops it on the wire.

| Topic | Shows | Colour |
|---|---|---|
| `.../viz/ego_boxes` | what ego detected itself | **greyscale**, lightness by local score — inverted, so confident is dark and doubtful is bright |
| `.../viz/peer_boxes` | every sender ego received this frame, **including gate-rejected ones**, one box per judged detection | **green** accepted (`matched`) / **yellow** pending (`deferred`) / **red** rejected (`uncorroborated`) |
| `.../viz/fused_boxes_markers` | the consensus estimate MS-PSF produced from both — "what I actually act on" — plus each peer's floating global-score label | **green** ≥ 0.8 / **yellow** 0.6–0.8 / **red** < 0.6, by the fused score |

Ego stays out of the hue palette because ego holds no verdict on itself; the peer hues are categorical (a decision, not a measurement) so they must not read as a gradient. Grey means "ego's own", so the three peer verdicts get the three hues; the fused layer reuses the same red/yellow/green traffic-light read, but thresholded on the continuous fused score rather than a discrete verdict.

A fused box sits at the fusion's **own** coordinates, near but not on top of the detections that produced it; which input fed which fused box is left to the eye (see `pipeline/fused_objects.py` for why the link isn't published).

**Shape is the class.** A vehicle draws as `assets/car.glb` or, when its reported length is ≥ `_BUS_MIN_LEN_M` (8 m), `assets/bus.glb`; a VRU as `assets/pedestrian.glb`; and anything else — including `unknown` — as a cylinder, deliberately neither. That is not cosmetic: the engine gates its kinematic scoring on `obj_type == vehicle` (`_vehicle_probs`), so a VRU and an `unknown` are judged under a different motion model than a car. `unknown` matters most — a class the sender could not place must not borrow a car's silhouette. A coach bus and a car share `obj_type == vehicle` on the wire, so length (the vehicle's fitted, most separated dimension) is what splits them; `_mesh_for_class` owns that rule. Each mesh is scaled to the size the sender reported (a vehicle on its length, a pedestrian on its height) and sits on its base rather than floating at the detection's geometric centre.

Meshes are referenced by `package://` URI, which **`foxglove_bridge` serves** from its default `asset_uri_allowlist` — the viewer fetches each one once, instead of the ~165 KB per object per frame an embedded `SceneUpdate` model would cost at this node's render rate. The cost of that choice: `package://` resolves only over a live bridge, so meshes do **not** render when a recorded MCAP is opened as a file.

**Labels say the class, plus the layer where naming it disambiguates one, and carry one score in the whole view** — the fused layer's, which is the number ego acts on. The fused label names no layer because `score:` appears on no other layer's label:

| Layer | Label |
|---|---|
| ego | `EGO \| Car` |
| peer | `AGENT 2 \| Pedestrian` |
| fused | `Car \| score: 0.87` |
| agent (floating above a peer's `ref_pos`, drawn on the **fused** layer) | `agent 2 \| global score: 0.62` |

`Car` / `Bus` / `Pedestrian` are display names local to this node; the wire vocabulary stays `perception_message.class_name_of`'s `vehicle` / `VRU` / `unknown` (`VRU` because senders group cyclists and motorcyclists into that class too, see `local_trust_estimation`'s `CLASS_OBJECT_TYPES`), and a class with no mesh falls back to it rather than being rounded into one of the named ones. `Bus` vs `Car` follows the same length split as the mesh, so the label always matches the silhouette. A per-box local score and the per-box verdict word were both dropped: a number on every object in two layers buried the fused one, and the verdict is already the box's colour. What a peer is worth stays on its agent label, where it annotates the agent it actually describes. That label goes out on the **fused** layer rather than `peer_boxes`, so hiding the per-object boxes to clear the view does not also take away what each agent is worth.

`peer_boxes.verdict_filter` narrows the peer layer to `pending_only` or `rejected_only`; it is read every render, so it can be changed live from Lichtblick's Parameters panel.

Markers are namespaced per sender (`agent_<id>`, `ego_<id>`), so individual senders can be toggled on and off in the panel.

**Frame.** Markers go out in the world frame the SDSM positions already live in (`frame_id`, default `world`). To view everything in one agent's sensor frame — the way the local trust pipeline renders — set Lichtblick's **Display frame** to e.g. `vehicle_lidar`; the converted bags ship the full `world → {agent}_base → {agent}_lidar` tree and TF does the transform, so this node needs no `tf2` of its own. The synthetic sim publishes no TF, so there the display frame is just `world`.

**Correlation, not synchronization.** The engine judges on a ~500 ms flush window, so a `TrustVerdicts` message arrives *after* the SDSM it judges. The node buffers raw messages by `(sender, msg_cnt)` and draws a sender only once its verdict has arrived **and** the two agree on detection count. That second check matters: a flush window that happens to contain two messages from one sender accumulates both into the engine while only the last is kept as the correlation key, so the verdict array can legitimately be longer than the correlated message's `num_objects` — drawing that would silently mis-colour boxes, so the frame is skipped instead. The same guard applies to `matched_ego_index` against `ego_msg_cnt`.

That correlation is also why the two input layers are **not contemporaneous on screen**: `ego_boxes` draws ego's *latest* frame, while `peer_boxes` draws the older frame the verdict names. That skew is real and is **not drawn**: the agent label used to report it as `dt`, and it was dropped because it is a property of this node's rendering rather than of the trust decision, so it sat beside the global score reading like something to act on. The pipeline's own pairing of the two frames is a separate question again, decided by `agent.flush_frame`'s window.

| Parameter | Default | Purpose |
|---|---|---|
| `ego_source_id` | `1` | Whose trust view to draw (live-settable) |
| `frame_id` | `world` | Frame the markers are published in |
| `stale_sec` | `2.0` | Drop cached messages and marker lifetime |
| `render_hz` | `4.0` | Marker republish rate |
| `marker_alpha` | `0.45` | Shape translucency |

### Adding to the sim scene

The whole scene — entities, motion, and visibility — lives in `sim_world.py`. To add a sensor or a target, edit the `_WORLD` table:

1. **Add an entity** — a new `_Entity` keyed by a fresh id, with its start position, travel heading (`None` if it never moves), `is_sensor` / `is_target` flags, object type, size, and (for a sensor) `equipment` type. A car is both a sensor and a target; the RSU is sensor-only; the VRU is target-only.
2. **(Optional) seed tick 0** — add the entity's id to another sensor's `_SEED_TARGETS` list if you want it reported on the very first tick; otherwise it appears as soon as the range + FOV + occlusion rules admit it.
3. **Run it** — if the new entity is a sensor, launch an agent for it (`-p agent_id:=<id>`) or add it to `_SENSORS` in the launch file. Nothing else needs changing: topics, trust engines, the N-way clustering, and the reputation DBs all key off `agent_id` automatically. A target-only entity (like the VRU) needs no node at all.

### 3. What to look for

On startup each node prints:

```
Agent 1 up: pub+sub on /sdsm, trust service /agent_1/get_trust_score, frame window=2000ms, R_default=0.5
```

Then every tick (`_TICK_INTERVAL_S`, currently ~0.74 s so the intersection scenario's full reveal sequence plays out in about a minute; 0.5 s in production), three kinds of lines confirm the loop is closed:

| Line | Meaning |
|---|---|
| `Published cnt=.. as agent_id=1 with 1 object(s)` | this agent is publishing |
| `Got cnt=.. from agent_id=2 with 1 object(s)` | **cross-traffic** — it hears the other agent |
| `agent=2 N=1 C=1 I=0 held=0 kine=0 attr=0 unc=0 S_frame=+1.000 F=1.000 V=1.000 R: 0.500 -> 0.600 [GATE FAIL - objects withheld (R_eff=0.500 < tau=0.708)]` | a **trust update** (gate verdict uses R_eff = R_old × freshness F × persistence penalty V, so a new agent fails the gate while its R climbs) |
| `Adding to database: {'01 00 00 00': [0.6, 0.614, ...]}` | batch written to SQLite every 10 frames |
| `frame 42 dropped, node kept alive (total=1): Traceback...` | **should never appear.** `flush_frame` is a timer callback, so an exception escaping the engine would otherwise take the whole node down for the rest of the run. The frame is skipped whole (no fused publish, no DB record, no reputation move) and the traceback logged at ERROR. A recurring one is a bug to fix, not a tolerable condition |

**Confirmation:** the `R:` value **climbs each frame** (0.50 → 0.60 → 0.70 → …) toward 1.0 while `tau` **drops**; once `R_old` clears `tau(R_old)` the verdict flips to `GATE PASS` and that agent's objects are admitted downstream. Whenever two sensors report the same road user, that object is `matched` for both (C rises, I=0), so their mutual reputations climb. An occasional `R` dip toward 0.5 just means no message landed in that frame window (silent-frame decay) — expected timing jitter.

### 4. (Optional) query the trust service

From a third terminal, ask agent 1 for its weighted trust in agent 2:

```bash
source install/setup.bash
ros2 service call /agent_1/get_trust_score \
  sdsm_interfaces/srv/GetTrustScore "{agent_id: 2, local_score: 1.0}"
```

The response `weighted_score` = R(agent 2) × `local_score`, so it rises as agent 2's
reputation climbs.

## Reputation database

Every agent node writes its reputation history to a SQLite database. `r_new` values are buffered in memory and flushed every `BATCH_SIZE` frame flushes (see `persistent_reputation_tracker.py`). On Ctrl+C the buffer is always written before exit, so no data is lost on a clean shutdown.

Each continuous interaction with an agent (from first contact to drop-off) is stored as one **session**. If an agent disappears for `SESSION_TIMEOUT` (3) or more consecutive frames and then reappears, a new session is created.

**Write-latency design decisions (edge deployment):**

- The connection runs `PRAGMA journal_mode=WAL` with `PRAGMA synchronous=NORMAL`: commits no longer fsync on every write, removing the largest per-frame storage stall. **Accepted tradeoff:** an OS crash or power loss can drop the last few committed batches; the database itself cannot corrupt (WAL guarantees atomicity), and a clean shutdown still flushes everything.
- `BATCH_SIZE` is 1 (write every flush) for testing. For a 20 Hz production tick, **10 is the recommended value**: one write every 0.5 s, keeping the crash-loss window at half a second. Larger batches buy almost nothing once the per-commit fsync is gone.
- `SESSION_TIMEOUT` is counted in *frames*. At a 20 Hz production rate the current value (3) means a 150 ms dropout already closes a session — it should likely become time-based before deployment.

### Location

Databases are written to a fixed location inside the repo, regardless of where the node is launched from:

```
<ros2_ws>/src/CPX-Mono/ros2/data/historical_reputations_<agent_id>.db
```

e.g. `historical_reputations_1.db` for agent 1. The path is derived from `COLCON_PREFIX_PATH`, so you must have sourced `install/setup.bash`. Override it with a ROS param:

```bash
ros2 run global_trust_perception agent \
  --ros-args -p agent_id:=1 -p db_path:=/some/abs/path.db -r __node:=agent_1
```

### Schema

```
sessions        (id, source_id TEXT)          — one row per interaction
reputation_log  (id, session_id, r_new, ts)   — one row per scored frame
```

`source_id` encodes all 4 bytes of the SDSM `source_id[4]` as space-separated hex bytes, e.g. `"02 00 00 00"` for agent 2. `ts` is the wall-clock Unix epoch (`time.time()`) captured when the value was scored, not when the batch was written; render it with `datetime(ts, 'unixepoch', 'localtime')`. Rows written before the column existed have `ts = NULL`; an agent whose **only** rows are NULL-ts reseeds at the 0.5 default on reappearance — its absence gap is unknowable, so history of unknown age is treated as no usable history (see `get_last_reputation_with_ts`).

### Querying the database

Use the helper script for a formatted table with column headers:

```bash
python3 src/CPX-Mono/ros2/src/global_trust_perception/scripts/show_reputations.py 1
```

The argument is the agent id; pass a `.db` path instead to point at a specific file.

To **clear** all stored reputation data for an agent (deletes every row, keeps the file):

```bash
python3 src/CPX-Mono/ros2/src/global_trust_perception/scripts/show_reputations.py --clear 1
```

To **clear all agent databases at once** (finds every `historical_reputations_*.db` in the data directory):

```bash
python3 src/CPX-Mono/ros2/src/global_trust_perception/scripts/show_reputations.py --clear-all
```

Or query directly:

```sql
-- full reputation history per agent, in order
SELECT s.source_id, s.id AS session, r.r_new,
       datetime(r.ts, 'unixepoch', 'localtime') AS recorded_at
FROM reputation_log r
JOIN sessions s ON r.session_id = s.id
ORDER BY s.source_id, s.id, r.id;

-- reputation trajectory for one session
SELECT r_new FROM reputation_log WHERE session_id = 1 ORDER BY id;

-- all sessions seen for a given agent
SELECT * FROM sessions WHERE source_id = '01 00 00 00';
```

## How matches / non-matches are handled
**R** = trust ranking assigned to another agent by ego car

| # | What happened | Bucket | Temporal state | Verdict | In `N_total`? | Effect on other car's reputation |
|---|---|---|---|---|---|---|
| 1 | Both cars see the object | `matched` | — (agreement is its own proof) | **Correct** (C) | ✅ yes | ⬆️ **raises** R |
| 2 | Only **your** car sees it | `ego_only` | any | **Held** | ❌ no | none (agents are not penalised for failing to see what ego sees) |
| 3 | Only the **other** car sees it | `other_only` | tracked (stable id) | **Deferred** (ledger) | later | held during the grace window from **first sighting**, then **back-paid Correct** if a peer corroborates it in time or **back-charged Incorrect** on expiry/death (see the ledger) |
| 4 | Only the **other** car sees it | `other_only` | untracked (no track id) | **Held** | ❌ no | none — without a stable id the ledger cannot follow the object across frames, so it is never graded |
| 5 | Other car sends **nothing** this frame | — | — | silent | ❌ no | ↔️ R **frozen** at its last value; its history stays in the reputation DB and seeds it on reappearance (decayed toward 0.5 by the time away — downward only) |

**Key design choices visible in the table:**

- **Agreement (row 1) is the main way to earn trust.** Matched objects are the only *instant* Correct, weighted by the reporter's certainty `c`.
- **A neighbor seeing more than you is judged over time, not instantly (row 3).** An `other_only` object is deferred: forgiven-and-rewarded if a reputable peer eventually corroborates it, penalized only if it stays uncorroborated past the grace deadline. This protects a good sensor with a genuinely unique vantage while still punishing a persistent fabrication.
- **Existence is taken at the sender's word (row 3).** The clock starts the frame a sender first reports an object; nothing waits for the tracker to confirm it. The local pipeline already dropped what its sender was unsure of, so a second existence filter here would only delay both the charge and the credit.
- **Agents are never penalised for what ego sees and they don't (row 2).** `ego_only` objects are always Held. Different fields of view, occlusion, and sensor range make this an unreliable signal to score against.
- **Verdicts are weighted by certainty.** Correct earns `c`. With uniform certainty 1.0 this is exactly integer counting.

## What a verdict does to reputation

`held` objects are excluded from `N_total`, so they neither raise nor lower
R. Only Correct/Incorrect count:

```
N_total = C + I
S_frame = (C - I) / N_total          # in [-1, +1]
R_new   = clip(R_old + DELTA * S_frame, 0, 1)   # DELTA = 0.10
```

If everything was held (`N_total == 0`) while the agent still sent data,
its reputation **decays toward the 0.5 baseline** instead of staying
frozen. A fully silent agent (row 5) is different: it is not scored at all,
so its reputation freezes until it reappears.

## Worked examples

- **Both cars track the same car ahead, every frame** → row 1 each frame →
  C=1, I=0, `S=+1` → R climbs (0.50 → 0.60 → 0.70 …).
- **Other car also reports a real object behind a building your car can't see**
  → that object is `other_only` (row 3), held from the frame it is first
  reported; once another car drives into view and corroborates it, every held
  frame is **back-paid as Correct** → rewarded, not penalized.
- **Other car keeps missing a car your car has tracked for seconds** → that
  object is `ego_only`; always Held (row 2) → no reputation effect.
- **Other car flashes a one-frame ghost** → `other_only` (row 3), whose track
  dies well inside the grace window → forgiven, no effect.

## Handling the persistent fake

A **persistent fake** (an attacker reporting the same ghost object every frame) produces a long-lived `other_only` track. The design does not forgive it forever and does not punish it instantly — because a high local score on an object no peer sees is equally the signature of a real object *occluded* from everyone else. Instead judgment is **deferred** (`deferred.PendingVerdicts`): the track is keyed by `(agent, track_id)` and held during a grace window (`T_DEADLINE_S` seconds from first sighting), on the assumption occlusion is transient. If a reputable peer corroborates it in time — reputation-mass support ≥ `SUPPORT_THRESHOLD_THETA` — every held frame is **back-paid as Correct** (the good sensor in a bad spot gets its full reward). If the deadline passes with no corroboration, every held frame is **back-charged as Incorrect** and each further ghost frame is charged instantly, draining the attacker's reputation. Colluders can't shortcut this: support is reputation *mass* scaled by each peer's reported certainty, so k low-reputation agents never reach the threshold, and every penalized frame lowers the reputation they could lend each other.

Occlusion false positives are bounded by the grace window (a genuinely occluded object is almost always corroborated once another vehicle drives into view) and by the output tiers, which mute-but-still-grade uncorroborated detections so an honest unique-vantage sensor never puts the vehicle at risk while it waits for a witness. Line-of-sight modeling (turning "nobody else sees it" into "a clear-view witness saw nothing") is future work.

## Running the MS-PSF fusion demo

The `scripts/mspsf_fusion_stub.py` script visualises how MS-PSF fuses detections from multiple agents, weighted by reputation. Run from `src/CPX-Mono/` after sourcing:

```bash
# prints fused-box table, no window (headless / container)
python3 ros2/src/global_trust_perception/scripts/mspsf_fusion_stub.py --no-draw

# opens an interactive 3D matplotlib window
python3 ros2/src/global_trust_perception/scripts/mspsf_fusion_stub.py

# saves the figure to mspsf_fusion_stub.png instead
python3 ros2/src/global_trust_perception/scripts/mspsf_fusion_stub.py --save
```

## Running the stage latency benchmark

`scripts/benchmark_stages.py` times every stage of the trust pipeline on synthetic frames and prints a per-stage table (mean / median / p95 / max in ms) plus an end-to-end total against the frame budget. Run from `src/CPX-Mono/` after sourcing:

```bash
# defaults: 20 agents x 256 objects each, 4 classes, 50 timed frames, 20 Hz budget
python3 ros2/src/global_trust_perception/scripts/benchmark_stages.py

# custom scale
python3 ros2/src/global_trust_perception/scripts/benchmark_stages.py --agents 10 --objects 32 --frames 300
```

`--agents` sets how many sending agents, `--objects` how many detections each reports per frame (the decode stage caps at the `SdsmPayload` array capacity, 256, read from the message rather than hardcoded), `--classes` how many distinct object classes (this drives the fusion `kappa` term — with a single class kappa is constant and the dominant stage is understated by roughly 10%), `--flush-hz` the target rate whose reciprocal is the per-frame budget, `--frames` / `--warmup` the sample count (warm-up frames absorb JIT/cache effects and are not timed), and `--seed` fixes the scene randomisation. The DB stage flushes every frame, i.e. it reports the worst-case write cadence regardless of `BATCH_SIZE`.

**SORT is timed but reported separately.** It runs in the tracker node, a different process, concurrently with `flush_frame` — so it appears in its own table and is deliberately absent from the end-to-end total, which would otherwise double-count it against a budget it does not spend.

### Finding the agent-count limit

`--agents-sweep` runs one point per agent count and reports `N_max`, the largest count whose **p95** frame still fits the budget:

```bash
python3 ros2/src/global_trust_perception/scripts/benchmark_stages.py \
    --agents-sweep 2 5 10 20 40 --dump-samples /tmp/global_samples.json
```

```
 agents   mean ms  median ms   p95 ms   max ms  verdict
      2     5.345      5.364    5.423    5.423  OK
      5    10.896     10.705   13.402   13.402  OK
     10    22.668     22.612   23.144   23.144  OK
     20    61.699     60.959   64.620   64.620  OVER
N_max: 10 agents sustain 20 Hz at p95.
```

p95 rather than mean decides the verdict: the flush timer has no queue to absorb a slow frame, so a stage that exceeds the window one frame in ten is already dropping that window however good its average looks.

`N_max` matters beyond capacity planning — it is the **validity boundary for any end-to-end latency figure**. Below it the frame buffer drains every window and latency has a steady state; above it the buffer grows every window and there is no steady-state number to quote.

`--dump-samples` writes the raw per-frame series as JSON for `tools/latency_model`. Raw series rather than summary statistics, because the model composes them with other terms and percentiles of a sum cannot be recovered from percentiles of its parts. The payload names which stages compose `e2e_ms` in `e2e_stages`, so a consumer cannot accidentally fold in SORT.

## End-to-end latency

Service time (what the benchmark measures) is not the same as the latency a detection experiences. The full path decomposes into four terms:

```
L_e2e  =  S_local  +  D_tx  +  W_batch  +  S_global
```

| Term | What it is | Where it comes from |
|---|---|---|
| `S_local` | local pipeline compute | `local_trust_estimation`'s benchmark, over a real bag |
| `D_tx` | send → receive | measured live (`pmsg.send_latency_of`) |
| `W_batch` | arrival → the flush that consumes it | bounded by the frame window; ~half of it on average |
| `S_global` | `flush_frame` compute | this benchmark, as a function of agent count |

Only `S_global` needs a multi-agent scene, which is why the absence of a multi-agent dataset does not block a latency figure — it means the figure is reported *as a function of agent count* rather than as one number.

A running agent measures two of these directly and logs them at debug level, alongside the flush duration it already reported:

```
[DIAG] latency terms over 240 message(s): D_tx mean=12.400ms max=31.000ms,
       W_batch mean=24.180ms max=49.700ms (window=50ms)
```

`W_batch` should sit near half the window; a mean drifting toward the *full* interval means senders are landing just after a flush and waiting nearly a whole one for the next. Ego's own echo is excluded — a self-published message has no transmission to measure.

**`D_tx` resolution is 1 ms**, the granularity of `sdsm_time_of_day_ms` on the wire. That is fine for a radio link where `D_tx` is tens of ms, but a loopback or localhost run will report `0.000ms` rather than a real sub-millisecond number. Treat `D_tx` as a swept parameter in the model rather than trusting a zero measured on loopback.

Finally, the local and global figures **do not sum into a single pipeline number**. The two stages run in separate processes at different rates; only `L_e2e` above composes them, and it does so through `W_batch`, not by addition.

## Package layout

Three areas, split by reason-to-change:

```
global_trust_perception/
├── pipeline/            I/O + orchestration: ROS nodes, the wire codec, the
│                        SQLite store, the sim, the renderer, and the per-frame
│                        orchestrator that composes everything below.
├── trust_calculations/  Trust policy: what counts as correct, what corroborates
│                        what, how reputation moves. Pure functions over counts,
│                        reputations, and cluster membership — no ROS, no
│                        geometry, no I/O.
└── mmcooper_fuse/       The vendored rotated-BEV fusion engine plus the one
                         boundary module that translates pipeline data into it.
```

Dependencies run one way — `pipeline` → `trust_calculations`, `pipeline` → `mmcooper_fuse`, and the read-only `lichtblick` viz nodes → `pipeline` (never the reverse) — and nothing in `trust_calculations` imports from `pipeline`, so the policy layer stays independently testable (most of the test suite runs without ROS on the path).

## Where this lives in the code

| Concern | Module | Key function/class |
|---|---|---|
| Foxglove/Lichtblick scene viz (read-only) | `lichtblick/scene_node.py` | `SceneNode` (`ros2 run global_trust_perception scene`) |
| Per-object trust markers (read-only) | `lichtblick/trust_view_node.py` | `TrustViewNode` (`ros2 run global_trust_perception trust_view`) |
| Lanelet map overlay (read-only) | `lichtblick/lanelet_overlay_node.py`, `lichtblick/geo_anchor.py` | `LaneletOverlayNode` (`ros2 run global_trust_perception lanelet_overlay`) |
| SORT tracking + TrackUpdate publisher | `global_trust_tracker/tracker_node.py` | `TrackerNode` (separate package) |
| Rotated-BEV cooperative fusion (MS-PSF) | `mmcooper_fuse/fusion.py`, `mmcooper_fuse/geometry.py` | `fuse_detections`, `mspsf` |
| N-way clustering (pipeline boundary) | `mmcooper_fuse/adapter.py` | `fuse`, `StreamInput`, `FusionResult` |
| Display derivation (z/height, contributors, bounds) | `mmcooper_fuse/display_derivation.py` | `fused_pose`, `scene_bounds`, `contributors_of` |
| Temporal confirmation (3D IoU + distance fallback) | `global_trust_tracker/SORT/modified_SORT_centroid.py` | `Sort` (in `global_trust_tracker` package) |
| Verdict policy (weighted C/I/held, pen, corroboration) | `trust_calculations/consistency.py` | `weighted_ego_consistency`, `pen`, `corroboration_support`, `is_corroborated` |
| Deferred other_only ledger (grace/back-pay/expiry) | `trust_calculations/deferred.py` | `PendingVerdicts` |
| Kinematic-Dynamic Consistency Score (KDS: fusion orientation + reputation) | `trust_calculations/consistency_checks/kinematic_checks.py` | `KinematicHistory.score_and_update` |
| Size-consistency score (SS, logged only) | `trust_calculations/consistency_checks/attribute_checks.py` | `check_size_agreement` |
| Reputation + threshold + output tiers | `trust_calculations/reputation.py` | `reputation_update`, `dynamic_threshold`, `trust_fixed_point`, `HIGH_TRUST` |
| Kinematic freshness gate factor F (tracked speed × message latency) | `trust_calculations/reputation_multipliers/kinematic_freshness.py`, `.../sender_motion.py` | `kinematic_freshness_factor`, `SenderMotionHistory` |
| Persistence penalty V (on-off attackers) | `trust_calculations/reputation_multipliers/persistence_penalty.py` | `PersistencePenalty` |
| Inter-session absence decay | `trust_calculations/reputation_multipliers/absence_decay.py` | `absence_decay` |
| Per-frame orchestration | `pipeline/trustworthy_perception.py` | `TrustEngine.process_frame` |
| Frame window: batching, per-sender dedup, latency probe | `pipeline/agent.py` | `AgentNode.flush_frame`, `_latest_per_sender`, `_RunningStat` |
| Diagnostic verdict channel (viz-only) | `pipeline/trust_verdicts.py` | `build`, `topic`, `ego_of`, `sender_of` |
| Message encode/decode | `pipeline/perception_message.py` | `build`, `get_global_positions_of`, `get_dims_of`, `get_local_scores_of`, `local_score_of`, `get_headings_of`, `sender_id_tuple`, `send_latency_of`, `with_global_score`, `global_score_of` |
| Reputation history (SQLite) | `pipeline/persistent_reputation_tracker.py` | `PersistentReputationTracker` |
| Live 3D visualization | `pipeline/visualization.py` | `visualise`, `draw_prism_3d`, `draw_box_3d` |

## References

- **Kinematic freshness decay** — adapted from R. Su, Y. Jin, Y.-Q. Song, *"A cooperative trust model addressing CAM- and CPM-based Ghost Vehicles in IoV"* (2024, hal-04453209), Eq. 1: their exponential decay weight `ρ^(t−tₙ)` over a stream of CAM messages, applied here to a *blind distance* (tracked speed × this message's own send→receive latency, `SenderMotionHistory` + `send_latency_of`) rather than elapsed time directly — deliberately keyed to this message's own latency, not the gap since the sender's previous one, so a sender silent for a while that then reports with negligible latency reads as fully fresh. The grace band, the floor, and deriving ρ from the trust gate's fixed point are this project's additions.
- **Absence decay** — this project's Solution-C baseline drift extended to wall-clock inter-session gaps, with the downward-only constraint (silence never raises reputation).
