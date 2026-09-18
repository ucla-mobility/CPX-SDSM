# consistency_checks

Two supplementary scores that run alongside the main trust pipeline in `TrustEngine.process_frame`, approximating CooperFuse Ch. 2 (Zheng, "Cooperative Perception for Safer Smart Intersections"). Both are continuous in `(0, 1]` / `[0, 1]`, not boolean. **KDS is also a fusion input** (computed before MS-PSF runs, see below) as well as a reputation signal; **SS is diagnostic-only**, same as its retired boolean predecessor.

---

## 1. Kinematic-Dynamic Consistency Score (`kinematic_checks.py`)

### What it does

Approximates CooperFuse eq 2.2-2.4: fits a minimum-jerk trajectory between a track's previous Kalman state (`X_track`, saved after each `Sort.update()`) and its newly reported position, and converts the required energy into a score. The full module docstring spells out exactly what's ported from the paper versus approximated; the summary:

| From the paper | Approximated instead |
|---|---|
| Minimize integral of squared jerk (eq 2.3) | — |
| Boundary conditions `x(0) ∈ X_track`, `x(T)` = candidate (eq 2.4) | — |
| Energy → score conversion (paper leaves this **unspecified**) | `KDS_jerk = exp(-J / J0(T))`, with `J0(T)` derived per frame period from a stated displacement tolerance (not the paper's) |
| Path threaded through Reed-Shepp waypoints `x(t_i) ∈ X_RS` (bounded turning radius, forward **and backward**) | **Dubins** path geometry — the forward-only special case of Reed-Shepp. `KDS_curvature = straight_line_distance / shortest_Dubins_length` between the previous heading and the candidate's reported heading, at a fixed turning radius. Reversal/`C\|C\|C` ("three-point-turn") path words were left out deliberately: their tangent-point geometry is materially harder to derive correctly than the 4 CSC words used here, which were verified by simulating the actual arc-line-arc path across 2000 random pose pairs (zero failures) — a bar the reversal words weren't held to, so they weren't shipped |

`KDS = KDS_jerk × KDS_curvature`. The two are **multiplicative, not summed into one energy**, because they live on incompatible natural scales: minimum-jerk energy for a point-to-point move scales as `720·(Δp)² / T⁵` (standard closed form), so metre-scale position errors produce energies in the 1e5–1e8 range over the frame periods this runs at, while the curvature ratio is dimensionless and bounded in `(0, 1]`. Summing them at any fixed weight would let one silently swamp the other; multiplying keeps each term's effect legible regardless of the other's magnitude.

`KDS_curvature` needs the candidate's **reported heading**, not just its position — a new input to `score_and_update()` (`headings`, J2735 degrees, parallel to the detection list). Without it (or with the J2735 "unavailable" sentinel), the term is neutral (`1.0`) — can't judge a heading change without a heading.

**Class gating is soft, not a hard switch.** MS-PSF's own §3.1.5-style evaluation is broken out by car/pedestrian/cyclist/truck, and pedestrian-vs-small-vehicle is exactly where a detector's class confidence is weakest — a hard "is it a car" argmax would apply the full curvature penalty to a misclassified pedestrian. Instead:

```
KDS_curvature_eff = 1 - P(vehicle) * (1 - KDS_curvature_raw)
```

`P(vehicle)` is synthesized from the per-detection class-probability vector `c_i` (`_vehicle_probs()` in `trustworthy_perception.py`, mirroring `mmcooper_fuse.fusion.build_class_probs`'s own per-class formula — the same synthesis MS-PSF itself uses for `c_i`, eq 3.19). Deliberately the **pre-fusion** `c_i`, not the fused `c*`: under Option B, KDS feeds the fusion weights that determine `c*` in the first place, so gating on `c*` would close that loop. `P(vehicle)=1` (full penalty) with no class data at all — the same conservative default used before this was wired up. The identical formula is applied in `mmcooper_fuse/fusion.py`'s `_orient_with_kds` (per-candidate, using `class_probs[group_arr]` directly, no synthesis needed there since fusion already has a real `class_probs` matrix) — that gate had the exact same hard-argmax problem and was fixed for consistency, not because it was separately reported.

### Calibration

What is tuned is a **tolerance on unexplained displacement**, in metres; the energy scale is derived from it per frame period rather than fixed. Under the constant-velocity boundary conditions used here the closed form collapses to `J = 720·d²/T⁵` with `d = |p₁ − p₀ − v₀T|`, so choosing `J0(T) = 720·d_tol(T)² / (T⁵·ln 2)` makes `KDS_jerk = 2^−(d/d_tol)²`, exactly `0.5` at `d = d_tol(T)` by construction.

```
d_tol(T) = _KDS_NOISE_M + ½ · _KDS_A_MAX_MS2 · T²      # 0.20 m, 6.0 m/s²
```

The two terms are the two reasons a truthful report misses its constant-velocity prediction, and they scale differently in `T`: `_KDS_NOISE_M` is the part that does **not** shrink with the frame period (the 0.1 m wire grid SDSM positions arrive on, `sdsm_units.OFFSET_UNIT_M`, plus centroid jitter), while `½aT²` is what a bounded but unmodelled acceleration can add on top of constant velocity.

**This replaced a fixed `_KDS_J0 = 130000.0`**, tuned so a 2 m displacement scored `0.5` at `dt ≈ 0.5s`. Fixing `J0` fixes an *energy*, and energy carries `T⁻⁵`, so the displacement it tolerated drifted as `T^2.5` — 197.8 cm at `dt=0.5s`, 9.7 cm at the replay pipeline's flush period, 5.4 cm at the senders' true frame period, i.e. *below one quantisation step of the wire format*. An honest, perfectly tracked object stepping one unit scored `0.089` and was demoted; measured on clean recorded data with no attackers, the check demoted **64 % of matched pairs**. Expressed as a velocity error the same constant swung from 0.13 m/s at `dt=0.05s` to 11.2 m/s at `dt=1.0s`. The tolerance above stays in a 1.6–4 m/s band across that whole range while still scoring a 0.5 m single-frame teleport at 0.05 and a 2 m one at 0.0 (`test_kinematic_checks.py` pins both directions).

`_MIN_TURN_RADIUS_M = 5.0` approximates a car's minimum turning radius.

### Cold start and missing data

A new track (no previous-frame history) or a too-small `dt` cannot be judged and scores the neutral `1.0` — same conservative default as the retired boolean check's `consistent = True`.

### Where the score is used

KDS is computed **before** MS-PSF fusion runs now (not after, as the retired boolean version was), because it is a fusion input:

1. **Fusion (`mmcooper_fuse/fusion.py`, `_orient_with_kds`)** — only affects the fused box's **orientation**, via a KDS-weighted circular mean across the group's candidate headings (eq 2.8's actual scope). Position and scale are fused exactly as before (the existing support/score/reliability-weighted corner average), untouched by KDS. Vehicle-class gated: non-vehicle groups fall back to a plain (unweighted) circular mean.
2. **Trust scoring, ego-matched path (`pipeline/trustworthy_perception.py`, Stage 3)** — multiplies into `matched_certainties` (`other_score × KDS`), so a kinematically-implausible matched pair contributes less to `C` (Correct) instead of being excluded outright. This is the direct descendant of the retired boolean version's demote-to-`ego_only` behaviour, made continuous.
3. **Trust scoring, other-agent-corroborated path (`pipeline/trustworthy_perception.py`'s `ledger_obs` → `trust_calculations/deferred.py`)** — a detection ego never sees, corroborated instead by other agents (or settled by the deferred ledger: `Corrob`/`CorrobPending`/`CorrobExpired`/`Vindicated`), is *also* KDS-discounted before it can add to `C`. This was initially missed — the retired boolean check never covered `other_only` either, so it was easy to carry that same scope forward by default rather than by principle. **Crucially, this discount is credit-only.** `deferred.PendingVerdicts` now tracks two separate accumulators per pending entry: `sum_certainty` (raw local certainty, drives every INCORRECT-side amount — `_backcharge` on expiry/death, `pen(c)` on drip) and `sum_credit` (certainty × KDS, drives every CORRECT-side amount — corroboration, back-pay, vindication). Naively discounting one shared value would have softened the penalty for exactly the ghosts that look most kinematically implausible; keeping them separate means KDS can only ever suppress unearned credit, never blunt a deserved charge.

These uses are independent — same underlying score, computed once, consumed by three separate paths; fusion never touches reputation, and neither reputation path touches fused geometry or each other.

---

## 2. Size-consistency score, SS (`attribute_checks.py`)

### What it does

Approximates CooperFuse eq 2.5: `SS = min(d_pre, d_curr) / max(d_pre, d_curr)`, elementwise over the object's dimensions, for each matched `(ego_idx, other_idx)` pair.

| From the paper | Approximated instead |
|---|---|
| `SS = min/max` ratio, elementwise over `(l, w, h)` (eq 2.5) | Height is **not** compared — this pipeline has never treated the height channel as reliable for cross-agent comparison (the retired boolean version discarded it too); SS is the ratio over `(width, length)` only |
| — (paper doesn't specify the elementwise → scalar reduction) | Reduced via `min` across axes — the worst-agreeing dimension bounds the score |

`check_size_agreement()` is now purely geometric — it takes no reputation parameter. Reputation gating (an agent's size agreement only counts once `other_rep` clears `_SIZE_REP_GATE = 0.70`, same threshold as the retired version's `_REP_GATE`) moved to the call site in `trustworthy_perception.py` Stage 2c, since it's a trust-policy decision, not a geometric one.

### What the score means (still, as before)

**SS does not feed reputation.** It is diagnostic-only, logged as `attr_correct` in `FrameStats`, exactly as the retired boolean verdict was. This was deliberately preserved, not changed: `matched_certainties` (the quantity that flows into `C`/`I`/`R_new`) is multiplied only by KDS. An earlier draft of this change also multiplied SS in there — a real bug, caught because it traps every fresh agent (`R_old = REPUTATION_DEFAULT = 0.5 ≤ _SIZE_REP_GATE`) at zero corroboration forever, since `_SIZE_REP_GATE` zeroed `SS` for exactly the reputation range new agents start in. Fixed by keeping SS out of `matched_certainties` entirely.

---

## Effect on `FrameStats`

`kine_flagged` and `attr_correct` are diagnostic-only counts derived by thresholding the continuous scores (`_KDS_FLAG_THRESHOLD = 0.5`, `_SS_AGREE_THRESHOLD = 0.8` in `trustworthy_perception.py`) — they no longer gate anything themselves; the actual math uses the continuous KDS/SS values directly.

| Field          | Type  | Meaning |
|---|---|---|
| `kine_flagged` | `int` | Number of matched pairs with `KDS < 0.5` this frame (diagnostic only — matched pairs are no longer demoted) |
| `attr_correct` | `int` | Number of matched pairs with `SS >= 0.8` AND `other_rep > _SIZE_REP_GATE` (diagnostic only) |

---

## What does and does not affect reputation

| Signal | Affects R? | How |
|---|---|---|
| Position match (Correct) | **Yes** | Direct C increment → S_frame → R_new |
| KDS, ego-matched path | **Yes** | Multiplies into `matched_certainties` → C → S_frame → R_new |
| KDS, other-agent-corroborated path (ledger) | **Yes** | Discounts the `sum_credit` accumulator only (corroboration/back-pay/vindication) → C. Never discounts `sum_certainty` (the penalty accumulator) — a kinematically-implausible ghost is never charged less than an equally-fabricated "smooth" one |
| KDS (fusion orientation weight) | **No** | Separate consumer of the same score; only affects the fused box's heading |
| SS (size agreement) | **No** | Logged only, same as before |

---

## Assumptions and limitations

- **Constant-velocity continuation.** KDS_jerk's minimum-jerk fit assumes the track resumes its previous velocity by `t=T` (boundary condition, not observed) — an object that genuinely accelerates or decelerates may score lower than it "should." The paper's own boundary conditions (`x(T) ∈ X_track`) don't specify final velocity either.
- **KDS_jerk is speed-independent, unlike the retired boolean check.** The retired check widened its tolerance for fast movers (`threshold = max(base_m, speed·dt·factor)`, up to 5 m of slack at 20 m/s vs. 2 m stationary). The jerk-energy formulation does not: verified numerically that the same "excess" displacement beyond constant-velocity continuation produces identical energy regardless of the track's speed (0, 5, 20, 40 m/s all gave the same `J` for the same 2 m surprise). Arguably more physically honest (the jerk needed to explain a given surprise doesn't depend on background velocity), but it's a deliberate, disclosed difference from the old policy, not a bug — reintroducing speed-scaling into the tolerance would restore it if wanted.
- **Dubins, not full Reed-Shepp.** `KDS_curvature` is the shortest of the 4 forward-only CSC (circle-straight-circle) Dubins path words — it does not include Reed-Shepp's reversal/`C\|C\|C` ("three-point-turn") words. This is a disclosed scope reduction, not an oversight: the CSC words used here were verified by simulating the actual arc-line-arc path across 2000 random pose pairs and confirming it reaches the goal pose (zero failures); the reversal words' tangent-point geometry could not be derived with the same confidence, so they were left out rather than shipped unverified. Practically: a candidate whose only plausible explanation involves backing up will register as more implausible (lower `KDS_curvature`) than a true Reed-Shepp planner would say, since Dubins can't find the shorter reversing path.
- **`dt` is the frame's, not each track's.** The energy scale now follows the frame period (see Calibration), so a change of tick rate no longer silently retunes the check. What remains is that `dt` comes from `process_frame`'s flush-to-flush gap, while the two positions being compared are consecutive *sensor frames* — close but not identical (0.15 s vs ~0.118 s on the scene_01 recordings). With the tolerance now set by a noise floor plus `½aT²`, that mismatch is worth a few centimetres of apparent displacement rather than a verdict; threading a true per-track capture gap would need a per-track timestamp in `TrackData`, deliberately not taken until it is measured to matter.
- **SORT tracks ego and other independently**, and ego itself is never kinematically self-checked (KDS only ever gated *other* agents' corroboration of ego) — ego's KDS defaults to the neutral `1.0` wherever it's needed as a fusion input.
- **Class gating is now genuinely live, both directions.** `agent.py` decodes J2735 `obj_type` unconditionally now (previously only when `self.visualize`, since labels fed the display panel only) and passes it into `process_frame()` as `classes_by_agent`/`ego_classes`. This feeds both `KinematicHistory.score_and_update`'s hard `P(vehicle)` gate (above — a direct label lookup, not a statistical synthesis) and `StreamInput.labels`, so MS-PSF's own `kappa` class-overlap gate (previously always vacuous — every detection collapsed to a single class column) now genuinely separates detections by class too: verified a pedestrian and a vehicle reported at nearly the same position no longer merge into one MS-PSF cluster.
- **Dimensions come from `get_dims_of()` format**: `(width, length, height)` tuples in metres. Height is ignored.
- **`_axial_circular_mean`'s fused heading is box geometry only — never a directional (front/back) heading, and nothing downstream currently treats it as one.** Halving the doubled-angle mean recovers the correct orientation only up to a 2-fold ambiguity (`φ` vs. `φ+π`); for a plain rectangle that's fine, since `φ` and `φ+π` describe the identical box, which is exactly the case eq. 2.8 and `_orient_with_kds` are handling. Verified by tracing every consumer: `_orient_with_kds`'s own output only ever feeds `corners_from_pose` (box geometry); `visualization.py`'s one call to `fused_pose()` on a fused box discards the yaw immediately and draws the plain corner prism, no arrow or oriented mesh; the trust-output rebroadcast republishes each sender's original, unmodified SDSM (heading included) and never touches `fuse()`'s output at all; every directional/front-back-sensitive rendering in the codebase (`scene_node.py`'s oriented car meshes, `visualization.py`'s per-detection arrows) is driven exclusively by each sender's own raw J2735 heading, never by fused geometry. **If this ever changes** — the fused output gets published rather than just displayed, or feeds a motion predictor/planner — a velocity-based front/back disambiguation step (front should point along the track's direction of travel) would need to be added on top at that point; axial data structurally cannot supply it, by construction.

  **Correction (2026-08-24): the audit above is right about consumers and
  wrong about the consequence.** It looked for a consumer that reads the fused
  yaw *directionally*, found none, and concluded the ambiguity is harmless.
  But the damage happens before orientation is read at all.
  `rotated_weighted_boxes_fusion` (`geometry.py:153`) is a bare `tensordot`
  over corner arrays with no correspondence alignment, and `adapter.py:161`
  rebuilds each candidate's corners *from its heading*. So two candidates that
  disagree by 180° have their corner lists rotated by two relative to each
  other, and averaging pairs every corner with the one diagonally across the
  box: the fused box keeps its centre and collapses to **zero length and
  width** (`pose_from_corners` then returns yaw exactly `0.0` on the
  degenerate result). It is an *extent* bug, not an orientation bug, and it
  needs no directional consumer to bite. On scene_01, 54.3% of matched pairs
  are opposed, because the autosense trackers never resolve front/back — each
  flips its own track between frames. Being axial is what makes
  `_axial_circular_mean` correct; it is the corner average upstream of it that
  is ill-posed. Mitigated at source by `sdsm_publisher`'s
  `heading_fold_seam_deg` (see `local_trust_estimation/README.md`), which
  leaves this file's math untouched.

---

## Validation plan (blocked on data/testing access)

Ordered by dependency — each step blocks the ones after it. Steps 2 and 4 are code-complete (this session); the rest need a real detection dataset / training split and can't be run here yet.

1. **[Partially done] Fix the energy→KDS map.** The paper's one genuine underspecification (§2.3.4 only says the energy from eq. 2.4 "is converted into a kinematic and dynamic consistency score"). Code uses `KDS_jerk = exp(-J / J0(T))`. The anchor to the retired boolean check's `_KINE_BASE_M` noise floor is **gone** — the scale now comes from a displacement tolerance stated in metres and derived per frame period (see Calibration), which is what stopped the check demoting 64 % of matched pairs on clean data. **Still open:** those two constants are argued from sensor resolution and a vehicle acceleration bound, not fitted — re-tuning against the jerk-energy distribution over clean tracks in a real training split (median or 75th percentile is a reasonable anchor) is still the better answer once data is available. Acceptance test: fix `P(vehicle)=1`, sweep `J` from real tracks, confirm KDS spans a usable fraction of `[0,1]` and isn't pinned at either end.
2. **[Done] Land the hard class gate.** `P(vehicle) ∈ {0,1}` from `obj_type`, synthesis dropped, both call sites (`kinematic_checks.py`, `fusion.py`'s `_orient_with_kds`). Float-typed signatures, so the optional affine floor (`beta + (1-beta)*[hard indicator]`, disclosed as an addition, not Zheng's) can be layered in later without an interface change — not added this pass.
3. **[Blocked] Re-baseline with the kappa fix, KDS off.** Non-negotiable per the person who wrote this plan: the empty-`labels` bug made MS-PSF's class-overlap term vacuous, so every number from before this session is pre-correctness-fix. Report car/pedestrian/cyclist/truck separately (matching a §3.1.5-style table) — the VRU columns should move on this fix alone, and that needs to be isolated before KDS enters the picture.
4. **[Done] Verify `_orient_with_kds` against eq. 2.8.** Circular mean via `atan2` (was already the case), and the BEV `π`-ambiguity resolved before averaging, via `_axial_circular_mean` (was missing — verified numerically: two candidates 185° apart naively averaged to -87.5°, nonsense; correct answer is ~0-180°). First fix (`_resolve_pi_ambiguity`, resolve each candidate against the seed's own heading as a reference) had a real vulnerability, caught on review: the seed is chosen by MS-PSF's `support` score (spatial/detection agreement), which has no relationship to heading quality, so a seed with a bad heading sitting near the ±90° ambiguity boundary relative to the group's true consensus could split a genuinely-agreeing cluster and corrupt the average — verified this collapses a tight cluster at 45° to ~135° (the bad reference's own value) when the reference sits at 135°. Replaced with `_axial_circular_mean`, the standard circular-statistics technique for axial (mod-π) data — double each angle, circular-mean, halve — which needs no reference at all and recovers ~45° regardless. Also confirmed: `KDS_curvature` is a curvature *proxy*, not the jerk integral of eq. 2.3-2.4 — that's fully captured by `KDS_jerk` alone; disclosed explicitly in the module docstring now.
5. **[Blocked] Measure KDS on, decomposed.** Primary metric AOE (the only thing eq. 2.8 touches). Watch AP and ATE/ASE for regressions — they shouldn't move if the position/scale decomposition is clean; if they do, orientation is leaking into the corner blend.
6. **[Blocked] Ablate against the alternative.** Run B-simple (`weights *= KDS` in the `r_{a_j}` slot of eq. 3.17, one multiplier on all corners) alongside the decomposed version. Also worth running `r_{a_j}` off with KDS on, since reliability weights are fixed priors and a per-detection consistency signal may subsume them.
7. **[Blocked] Log KDS by track speed.** Cheap once real tracks are available. Tells you whether the zero-jerk-at-rest issue is live (13/55 of Zheng's scenarios are stopping-at-signal). If stopped vehicles pin the top of the range, deadband small `J` to neutral — fine to leave disclosed rather than fixed until measured.

### Provenance

Eqs. 2.1-2.9 and 3.9-3.20 are Zheng's. The energy→KDS map, the class gate (hard, and the not-yet-added affine floor), any Stage-5 penalty term, and the whole idea of routing KDS into MS-PSF are this session's additions, not the paper's. §3.1.8 naming frame-level fusion as an open limitation is the citation that justifies this work existing at all.
