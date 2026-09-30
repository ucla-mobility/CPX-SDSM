# Ported verbatim (pure Python, no ROS deps) from CPX-Mono's
# ros2/src/global_trust_perception/global_trust_perception/trust_calculations/consistency.py
# — see that repo for the full design writeup. No logic changed.
"""
Ego-consistency policy: the one place that decides what makes a report
correct, incorrect, or held, and how much each verdict weighs.

This module owns the whole verdict POLICY and nothing else:
  - the tunable constants (penalty floor, support threshold, grace deadline),
  - pen(c)            : the confidence-weighted penalty for a wrong report,
  - corroboration_support() : the peer reputation mass behind each detection,
  - is_corroborated() : whether that support clears the bar,
  - weighted_ego_consistency() : the stateless matched/ego_only routing.

It is pure and geometry-free. The stateful other_only handling (grace,
back-pay, expiry) lives in deferred.PendingVerdicts, which imports the
deadline, pen(), and is_corroborated() from here so the policy has a single
definition. The orchestrator composes the two.

WHY weighted (vs the old integer C/I/held): local scores now carry per-object
certainty (occlusion / range aware), so a confident agreement should earn more
than a hesitant one, and — via the AFFINE FLOOR — a mistake must always cost at
least PENALTY_FLOOR_BETA regardless of how low the reporter set its confidence,
closing the "hedge the lie with a low score" dodge (adversarial trust
poisoning). Correct earns c; incorrect costs pen(c) = beta + (1-beta)*c, which
is >= beta AND increasing in c, so a hesitant mistake still costs more than a
hesitant correct earns.
"""

# --- Verdict policy constants (owned here; imported by deferred + orchestrator)
PENALTY_FLOOR_BETA = 0.5       # affine floor: pen(c) = beta + (1 - beta) * c
SUPPORT_THRESHOLD_THETA = 1.0  # reputation mass that corroborates (~two neutral
                               # witnesses, or one perfectly-reputable one); also
                               # the availability-guard bar
T_DEADLINE_S = 15.0            # grace window in SECONDS before an uncorroborated
                               # other_only track expires to Incorrect.
                               # Stated in wall-clock time, not flushes, because
                               # what it models is how long a real object can
                               # plausibly stay occluded from every peer -- a
                               # property of traffic, not of the flush rate.
                               # Counted in flushes (as the retired 20-flush
                               # constant was) it silently meant 10 s in sim
                               # (2 Hz) and 1 s in replay (20 Hz); at 1 s an
                               # honest sensor with a unique vantage is charged
                               # as a fabricator before any peer could possibly
                               # drive into view. TrustEngine converts using its
                               # own flush_hz.
T_DEADLINE_FLUSHES = 30        # fallback for a caller with no rate to convert
                               # from; prefer T_DEADLINE_S wherever flush_hz is
                               # known. Kept equal to T_DEADLINE_S at the sim's
                               # 2 Hz flush rate, so the two constants cannot
                               # describe different windows for the same scene.
                               # A caller that actually runs at 20 Hz gets
                               # 300 flushes for the same 15 s, computed by
                               # TrustEngine from its own rate -- which is why
                               # this fallback is a last resort, not a target.


def _clip01(x: float) -> float:
    """Clamp a certainty to [0, 1]."""
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else float(x)


def _certainty_of(scores_by_key: dict, key: object, det: int) -> float:
    """One stream's certainty for one detection; 1.0 when it reported none."""
    scores = scores_by_key.get(key)
    if scores is None or det >= len(scores):
        return 1.0
    return _clip01(scores[det])


def pen(c: float) -> float:
    """Affine-floor penalty for a wrong report of certainty c.

    pen(c) = PENALTY_FLOOR_BETA + (1 - PENALTY_FLOOR_BETA) * c, so pen(0) = beta
    (no dodge via under-confidence) and pen(1) = 1 (a confident lie costs the
    maximum). Strictly increasing in c.
    """
    return PENALTY_FLOOR_BETA + (1.0 - PENALTY_FLOOR_BETA) * _clip01(c)


def corroboration_support(clusters: list,
                          reliability_by_key: dict,
                          scores_by_key: dict,
                          ego_key: object,
                          peer_cap: float = 1.0) -> dict:
    """Certainty-weighted reputation mass corroborating each detection.

    clusters           : [{stream_key: det_idx}] from the phase-1 fusion — at
                         most one detection per stream per real-world object.
    reliability_by_key : {stream_key: reputation} as fed to that fusion.
    scores_by_key      : {stream_key: [certainty per detection]}, length-matched
                         to that stream's positions (missing -> 1.0).
    ego_key            : ego's stream key. Never counted as a corroborator:
                         ego presence makes a detection matched rather than
                         other_only, so the matched path scores it instead.

    Each peer p in a cluster contributes R_p * c_p, so a hesitant witness
    corroborates less than a confident one at the same reputation, and support
    stays reputation MASS rather than a head count — k low-reputation colluders
    never reach SUPPORT_THRESHOLD_THETA between them, and every penalised frame
    lowers the reputation they could lend each other.

    Geometry-free by construction: this reads cluster MEMBERSHIP, never boxes.
    Which detections share a cluster is the fusion's business; what that
    membership is worth as corroboration is this module's.

    Returns {stream_key: {det_idx: support}}. A detection in no cluster (or in
    one with no peers) simply has no entry; callers read it as 0.0.
    """
    support: dict = {}
    for cluster in clusters:
        for key, det in cluster.items():
            peers = set(cluster) - {key, ego_key}
            # peer_cap bounds what ONE peer can contribute, so a single (or a
            # single colluding pair of) high-reputation peers cannot reach the
            # threshold alone; 1.0 = uncapped (original behaviour).
            support.setdefault(key, {})[det] = sum(
                min(reliability_by_key.get(p, 0.0)
                    * _certainty_of(scores_by_key, p, cluster[p]), peer_cap)
                for p in peers
            )
    return support


def is_corroborated(support: float, theta: float = None) -> bool:
    """True iff peer reputation mass reaches the corroboration threshold (theta,
    default SUPPORT_THRESHOLD_THETA)."""
    return support >= (SUPPORT_THRESHOLD_THETA if theta is None else theta)


def weighted_ego_consistency(
    matched_certainties: list,
    ego_only: list,
) -> tuple[float, float, int]:
    """
    Stateless part of the frame verdict: matched + ego_only buckets.

    matched_certainties : c (the other agent's score) for each ego-matched
                          detection -> each adds c to Correct (cross-agent
                          agreement, weighted by the reporter's confidence).
    ego_only            : ego_certainty for each object ego reported that the
                          other agent did NOT. All items go to Held — agents
                          are not penalised for failing to see what ego sees.

    other_only is deliberately absent: it needs temporal state and is settled by
    deferred.PendingVerdicts, whose deltas the orchestrator adds to these counts.

    Returns (C, I, held). I is always 0.0 from this function; the only source
    of I is the deferred ledger's back-charges added by the caller. held entries
    are excluded from N_total = C + I.
    """
    C = float(sum(_clip01(c) for c in matched_certainties))
    held = len(ego_only)
    return C, 0.0, held
