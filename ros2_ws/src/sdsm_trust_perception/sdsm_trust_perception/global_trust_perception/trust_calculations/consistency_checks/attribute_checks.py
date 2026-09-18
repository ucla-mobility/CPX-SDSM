# Ported verbatim (pure Python, no ROS deps) from CPX-Mono's
# ros2/src/global_trust_perception/global_trust_perception/trust_calculations/consistency_checks/attribute_checks.py
# — see that repo for the full design writeup. No logic changed.
"""
Cross-agent size-consistency score (SS) for WBF-matched object pairs.

Approximates CooperFuse Ch. 2 eq 2.5 (Zheng, "Cooperative Perception for
Safer Smart Intersections"): for each object that ego and another agent both
reported (i.e., in matched after kinematic filtering), this module compares
reported object dimensions and returns a continuous agreement score.

FROM THE PAPER (eq 2.5):
    SS = min(d_pre, d_curr) / max(d_pre, d_curr), elementwise over
    d = [l, w, h]^T, giving a value in (0, 1] per axis.

APPROXIMATED, NOT PORTED:
    - height is not compared. get_dims_of()'s native dims only carry
      (width, length, height), and this pipeline has never treated the
      height channel as reliable for cross-agent comparison (the previous
      boolean version discarded it too); SS here is the elementwise ratio
      over (width, length) only.
    - the paper does not specify how the per-axis (l, w, h) ratios reduce to
      a single SS; this module takes the min across axes -- the worst-
      agreeing dimension bounds the score.

Reputation is NOT considered here -- this module is purely geometric
agreement. Whether/how much a given agent's reputation should discount that
agreement is a trust-policy decision made by the caller.

Dimensions are expected in get_dims_of()'s native format: (width, length, height).

CPX-SDSM NOTE: sdsm_codec.get_dims_of() supplies (0,0,0) whenever the
detected object carries no usable size — a VRU/obstacle in the current sim
JSON, which never sets vehicle size for those kinds (see sdsm_codec docstring).
_axis_ratio treats both-near-zero as agreement (1.0), so an all-VRU frame
scores this check as neutral rather than penalising or rewarding it.
"""


def check_size_agreement(
    matched_pairs: list[tuple[int, int]],
    ego_dims: list,
    other_dims: list,
) -> list[float]:
    """
    Return a continuous SS in (0, 1] for each matched (ego_idx, other_idx)
    pair: 1.0 = identical reported width and length, lower = greater
    disagreement. Out-of-range indices (missing dims) score 1.0 -- can't
    judge, matches the neutral default used elsewhere in this pipeline for
    missing data.

    matched_pairs : list of (ego_idx, other_idx) derived from the N-way
                    clusters, already filtered by kinematic_checks
    ego_dims      : list of (width, length, height) tuples for ego's detections
    other_dims    : list of (width, length, height) tuples for the other agent
    """
    results: list[float] = []
    for ego_idx, other_idx in matched_pairs:
        if ego_idx >= len(ego_dims) or other_idx >= len(other_dims):
            results.append(1.0)
            continue

        ew, el, _ = ego_dims[ego_idx]
        ow, ol, _ = other_dims[other_idx]

        results.append(min(_axis_ratio(ew, ow), _axis_ratio(el, ol)))

    return results


def _axis_ratio(a: float, b: float) -> float:
    """min(a,b)/max(a,b) in (0, 1]; both near-zero counts as agreement."""
    hi = max(a, b)
    if hi < 0.01:
        return 1.0
    lo = max(min(a, b), 0.0)
    return lo / hi
