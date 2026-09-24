"""Standalone MMCooperFuse aggregation algorithms for ROS2 runtime use."""

# ======================== CHANGED (kds-orientation) =========================
# Eo piece lives in the two helpers' docstrings rather than being repeated
# at every marker. The separate `CHANGED (vectorized)` block in mspsf is an
# unrelated, output-preserving rewrite -- see its own comment.
# ====================== END CHANGED (kds-orientation) =======================

from __future__ import annotations

import math  # CHANGED (kds-orientation)
from dataclasses import dataclass

import numpy as np
from scipy.special import softmax

from .geometry import (
    compute_iou,
    compute_self_iou_mat,
    corners_from_pose,  # CHANGED (kds-orientation)
    pose_from_corners,  # CHANGED (kds-orientation)
    rotated_weighted_boxes_fusion,
    to_polygon,
)

# CHANGED (kds-orientation)
# J2735 obj_type convention used elsewhere in this codebase (scene_node.py,
# sim_world.py): 1 = vehicle. KDS-weighted orientation (eq 2.8) assumes a
# car-like bounded turning radius, so low-P(vehicle) candidates are blended
# toward a plain (unweighted) circular mean instead -- see _orient_with_kds.
_VEHICLE_CLASS_IDX = 1


@dataclass
class FusionConfig:
    aggregation_method: str = "psa"
    nms_iou_threshold: float = 0.15
    score_threshold: float = 0.0
    mspsf_tau_fuse: float = 0.3
    mspsf_tau_graph: float = 0.05
    mspsf_kappa_thr: float = 0.3
    mspsf_temperature: float = 0.1
    mspsf_alpha: float = 0.4
    mspsf_gamma_agent: float = 0.2
    mspsf_gamma_modality: float = 0.3
    mspsf_lambda_modality: float = 0.5
    mspsf_delta_loc: float = 0.0
    mspsf_distance_threshold: float = 10.0
    mspsf_total_modalities: int = 3


def nms_rotated(boxes: np.ndarray, scores: np.ndarray, threshold: float) -> np.ndarray:
    """Greedy rotated-box NMS."""
    boxes = np.asarray(boxes, dtype=np.float32).reshape(-1, 4, 2)
    scores = np.asarray(scores, dtype=np.float32).reshape(-1)
    if boxes.shape[0] == 0:
        return np.array([], dtype=np.int64)
    polygons = np.array([to_polygon(box) for box in boxes], dtype=object)
    ixs = scores.argsort()[::-1]
    keep = []
    while len(ixs) > 0:
        i = int(ixs[0])
        keep.append(i)
        if len(ixs) == 1:
            break
        ious = compute_iou(polygons[i], polygons[ixs[1:]])
        remove_ixs = np.where(ious > threshold)[0] + 1
        ixs = np.delete(ixs, remove_ixs)
        ixs = np.delete(ixs, 0)
    return np.array(keep, dtype=np.int64)


def non_maximum_suppression(
    boxes: np.ndarray,
    scores: np.ndarray,
    threshold: float = 0.15,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Apply NMS and return boxes, scores, and selected original indices."""
    keep = nms_rotated(boxes, scores, threshold)
    return boxes[keep], scores[keep], keep


def psa(boxes: np.ndarray, scores: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Promote-Suppress Aggregation selection from the original codebase."""
    boxes = np.asarray(boxes, dtype=np.float32).reshape(-1, 4, 2)
    scores = np.asarray(scores, dtype=np.float32).reshape(-1)
    if len(boxes) == 0:
        return boxes, scores, np.array([], dtype=np.int64)

    iou_mat = compute_self_iou_mat(boxes)
    clusters = []
    visited = set()
    selected = []
    for idx, ious in enumerate(iou_mat):
        if idx in visited:
            continue
        neighbor_idxs = np.nonzero(ious)[0]
        clusters.append(neighbor_idxs)
        visited.update(int(i) for i in neighbor_idxs)

    for cluster in clusters:
        sub_iou_mat = iou_mat[np.ix_(cluster, cluster)]
        sub_scores = scores[cluster]
        values = sub_iou_mat.dot(sub_scores)
        bools = softmax(values / 1e-6) > 0.5
        selected.extend(cluster[bools])

    selected = np.array(selected, dtype=np.int64)
    return boxes[selected], scores[selected], selected


def build_class_probs(
    class_labels: np.ndarray,
    scores: np.ndarray,
    num_classes: int,
) -> np.ndarray:
    """Build soft class distributions from hard labels and confidence scores."""
    labels = np.asarray(class_labels, dtype=np.int64).reshape(-1)
    scores = np.asarray(scores, dtype=np.float32).reshape(-1)
    num_classes = int(max(num_classes, 1))
    if num_classes <= 1:
        return np.ones((len(labels), 1), dtype=np.float32)
    class_probs = np.zeros((len(labels), num_classes), dtype=np.float32)
    for i, label in enumerate(labels):
        k = int(np.clip(label, 0, num_classes - 1))
        class_probs[i, k] = scores[i]
        residual = (1.0 - scores[i]) / max(num_classes - 1, 1)
        for j in range(num_classes):
            if j != k:
                class_probs[i, j] = residual
    return class_probs


def _axial_circular_mean(theta: np.ndarray, w: np.ndarray) -> float:  # CHANGED (kds-orientation)
    """Weighted circular mean of angles that are only defined mod pi -- a
    BEV box's orientation is axial, not vectorial: rotating a rectangle
    180deg yields the identical box, so two candidates that agree on the
    box's orientation but disagree on which end is the "front" must not
    partially cancel in a plain circular mean instead of reinforcing (e.g.
    naively averaging 0deg and 185deg gives ~-87deg: nonsense, since the two
    candidates essentially agree, up to the pi ambiguity, on ~0/180deg).

    Standard circular-statistics technique for axial data: double each angle
    (mapping mod-pi data onto ordinary mod-2pi data), take the usual
    weighted circular mean, then halve the result.

    This replaces an earlier approach that resolved the ambiguity against a
    single reference candidate (flip anything more than 90deg from it) --
    that was reference-dependent, and a bad reference could corrupt the
    whole group's result, not just relabel it onto an equivalent branch:
    verified numerically that a tightly-agreeing cluster near 45deg,
    resolved against a bad reference at 135deg (90deg away -- exactly the
    worst case, sitting right on the ambiguity boundary), collapsed to
    ~135deg (the bad reference's own value) under the old method, while
    the doubling method used here recovers ~45deg regardless of any
    reference, because it never picks one.
    """
    two_theta = 2.0 * theta
    phi_hat = math.atan2(
        float(np.sum(w * np.sin(two_theta))),
        float(np.sum(w * np.cos(two_theta))),
    )
    return phi_hat / 2.0


def _orient_with_kds(  # CHANGED (kds-orientation)
    fused_box: np.ndarray,
    cand_boxes: np.ndarray,
    cand_kds: np.ndarray,
    cand_vehicle_probs: np.ndarray,
) -> np.ndarray:
    """Override fused_box's orientation with a KDS-weighted circular mean of
    the group's candidate headings (eq 2.8); position and scale are left as
    the weighted corner-average already produced them.

    Vehicle-class gated on the raw reported label (cand_vehicle_probs, in
    {0.0, 1.0} in practice -- see kinematic_checks.py's module docstring for
    why this is a hard switch on the label rather than a statistical blend):
    w_eff = 1 - P(vehicle)*(1 - KDS), the same formula kinematic_checks.py
    uses for the reputation-side term. Float-typed so an affine floor could
    be layered in later without an interface change.
    """
    p_vehicle = cand_vehicle_probs.astype(np.float64)
    kds_eff = 1.0 - p_vehicle * (1.0 - cand_kds.astype(np.float64))
    w = np.maximum(kds_eff, 0.0)
    w_sum = float(w.sum())
    if w_sum <= 1e-9:
        w = np.ones(len(cand_kds))
        w_sum = float(w.sum())
    w = w / w_sum

    cand_theta = np.array([pose_from_corners(b)[4] for b in cand_boxes])
    theta_hat = _axial_circular_mean(cand_theta, w)
    x_hat, y_hat, l_hat, w_hat, _ = pose_from_corners(fused_box)
    return corners_from_pose(x_hat, y_hat, l_hat, w_hat, theta_hat)


def mspsf(
    boxes: np.ndarray,
    scores: np.ndarray,
    class_probs: np.ndarray,
    agents: np.ndarray,
    modalities: np.ndarray,
    cfg: FusionConfig,
    agent_reliabilities: np.ndarray | None = None,
    kds_scores: np.ndarray | None = None,  # CHANGED (kds-orientation)
    vehicle_probs: np.ndarray | None = None,  # CHANGED (kds-orientation)
) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[np.ndarray]]:
    """Multi-Source Promote-Suppress Fusion for ROS detections.

    kds_scores    : optional per-detection Kinematic-Dynamic Consistency
                    Score (CooperFuse eq 2.8), same length as boxes/scores.
                    Affects ONLY the fused orientation within a group (see
                    _orient_with_kds) -- position, scale, and the fusion
                    weight itself (support/seed selection,
                    agent_reliabilities) are unaffected, matching eq 2.8's
                    orientation-only scope.
    vehicle_probs : optional per-detection P(vehicle) in {0.0, 1.0}, from
                    the raw reported label (see kinematic_checks.py's module
                    docstring for why this is a hard switch, not a
                    statistical synthesis off class_probs). None -> every
                    detection treated as a vehicle, the same conservative
                    default used elsewhere when class data is unavailable.
    """
    boxes = np.asarray(boxes, dtype=np.float32).reshape(-1, 4, 2)
    scores = np.asarray(scores, dtype=np.float32).reshape(-1)
    class_probs = np.asarray(class_probs, dtype=np.float32).reshape(len(boxes), -1)
    agents = np.asarray(agents, dtype=np.int64).reshape(-1)
    modalities = np.asarray(modalities, dtype=np.int64).reshape(-1)
    n = len(boxes)
    if n == 0:
        return boxes, scores, class_probs, []

    iou = compute_self_iou_mat(boxes, dist_threshold=cfg.mspsf_distance_threshold)
    # ========================= CHANGED (vectorized) =========================
    # affinity_boost and kappa replace two O(n^2) Python double-loops with
    # broadcast numpy ops. Diagonals match the loops: bonus is 0 on the
    # diagonal (i == j never differs), so affinity_boost stays 1.0 there, and
    # kappa's diagonal is pinned to 1.0 exactly as the np.ones init did.
    bonus = (
        cfg.mspsf_gamma_agent * (agents[:, None] != agents[None, :])
        + cfg.mspsf_gamma_modality * (modalities[:, None] != modalities[None, :])
    )
    affinity_boost = (1.0 + bonus).astype(np.float32)
    affinity = iou * affinity_boost

    kappa = np.minimum(class_probs[:, None, :], class_probs[None, :, :]).sum(axis=2)
    np.fill_diagonal(kappa, 1.0)
    # ======================= END CHANGED (vectorized) =======================

    edge = (iou >= cfg.mspsf_tau_graph) & (kappa >= cfg.mspsf_kappa_thr)
    np.fill_diagonal(edge, True)

    visited = np.zeros(n, dtype=bool)
    components = []
    for i in range(n):
        if visited[i]:
            continue
        stack = [i]
        visited[i] = True
        comp = []
        while stack:
            u = stack.pop()
            comp.append(u)
            for v in np.where(edge[u])[0]:
                if not visited[v]:
                    visited[v] = True
                    stack.append(int(v))
        components.append(np.array(comp, dtype=np.int64))

    support = np.zeros(n, dtype=np.float32)
    for i in range(n):
        mask = edge[i].copy()
        mask[i] = False
        if not np.any(mask):
            support[i] = cfg.mspsf_alpha * scores[i]
            continue
        nbr_aff = affinity[i, mask]
        nbr_scores = scores[mask]
        degree = float(nbr_aff.sum())
        raw = float((nbr_aff * nbr_scores).sum() / (degree + 1e-9))
        strength = min(1.0, degree)
        support[i] = cfg.mspsf_alpha * scores[i] + (1.0 - cfg.mspsf_alpha) * strength * raw

    reliabilities = None
    if agent_reliabilities is not None and len(agent_reliabilities) > 0:
        reliabilities = np.asarray(agent_reliabilities, dtype=np.float32).reshape(-1)

    # CHANGED (kds-orientation)
    kds = None
    if kds_scores is not None and len(kds_scores) == n:
        kds = np.asarray(kds_scores, dtype=np.float32).reshape(-1)

    veh_probs = None
    if vehicle_probs is not None and len(vehicle_probs) == n:
        veh_probs = np.asarray(vehicle_probs, dtype=np.float32).reshape(-1)
    elif kds is not None:
        veh_probs = np.ones(n, dtype=np.float32)  # no class data -> assume vehicle

    total_modalities = max(int(cfg.mspsf_total_modalities), 2)
    out_boxes = []
    out_scores = []
    out_class_probs = []
    out_groups: list[np.ndarray] = []

    for comp in components:
        unpicked = set(comp.tolist())
        while unpicked:
            seed = max(unpicked, key=lambda idx: support[idx])
            group = [seed]
            for j in unpicked:
                if j == seed:
                    continue
                cross_agent = agents[seed] != agents[j]
                tau = cfg.mspsf_tau_fuse * (1.0 - cfg.mspsf_delta_loc) if cross_agent else cfg.mspsf_tau_fuse
                if iou[seed, j] >= tau and kappa[seed, j] >= cfg.mspsf_kappa_thr:
                    group.append(j)
            group_arr = np.array(group, dtype=np.int64)

            logits = support[group_arr] / max(cfg.mspsf_temperature, 1e-8)
            logits -= float(np.max(logits))
            weights = np.exp(logits)
            weights = weights / (float(weights.sum()) + 1e-9)
            weights = weights * scores[group_arr]
            if reliabilities is not None:
                weights = weights * reliabilities[np.clip(agents[group_arr], 0, len(reliabilities) - 1)]
            weights = weights / (float(weights.sum()) + 1e-9)

            fused_box = rotated_weighted_boxes_fusion(boxes[group_arr], weights)
            if kds is not None and len(group_arr) > 1:  # CHANGED (kds-orientation)
                fused_box = _orient_with_kds(
                    fused_box, boxes[group_arr], kds[group_arr], veh_probs[group_arr]
                )
            fused_class = np.tensordot(weights, class_probs[group_arr], axes=1)
            fused_score = float(np.dot(weights, scores[group_arr]))

            grp_modalities = modalities[group_arr]
            grp_agents = agents[group_arr]
            unique_modalities = np.unique(grp_modalities)
            unique_agents = np.unique(grp_agents)
            h_src = 0.0
            if len(unique_modalities) > 1:
                fracs = np.array([np.mean(grp_modalities == q) for q in unique_modalities])
                h_src = float(-np.sum(fracs * np.log(fracs)) / np.log(total_modalities))
            elif len(unique_agents) > 1:
                n_agents = max(len(np.unique(agents)), 2)
                fracs = np.array([np.mean(grp_agents == q) for q in unique_agents])
                h_src = float(-np.sum(fracs * np.log(fracs)) / np.log(n_agents))
            if h_src > 0.0:
                p = float(np.clip(fused_score, 1e-6, 1.0 - 1e-6))
                logit = np.log(p / (1.0 - p))
                fused_score = float(1.0 / (1.0 + np.exp(-(logit + cfg.mspsf_lambda_modality * h_src))))

            out_boxes.append(fused_box.astype(np.float32))
            out_scores.append(fused_score)
            out_class_probs.append(fused_class.astype(np.float32))
            out_groups.append(group_arr)
            unpicked -= set(group_arr.tolist())

    return (
        np.stack(out_boxes, axis=0),
        np.array(out_scores, dtype=np.float32),
        np.stack(out_class_probs, axis=0),
        out_groups,
    )


def fuse_detections(
    boxes: np.ndarray,
    scores: np.ndarray,
    classes: np.ndarray,
    agents: np.ndarray,
    modalities: np.ndarray,
    cfg: FusionConfig,
    class_probs: np.ndarray | None = None,
    agent_reliabilities: np.ndarray | None = None,
    kds_scores: np.ndarray | None = None,  # CHANGED (kds-orientation)
) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[np.ndarray]]:
    """Dispatch the configured aggregation method."""
    boxes = np.asarray(boxes, dtype=np.float32).reshape(-1, 4, 2)
    scores = np.asarray(scores, dtype=np.float32).reshape(-1)
    classes = np.asarray(classes, dtype=np.int64).reshape(-1)
    method = cfg.aggregation_method.strip().lower().replace("_", "-")

    if cfg.score_threshold > 0.0:
        keep = scores >= cfg.score_threshold
        boxes, scores, classes = boxes[keep], scores[keep], classes[keep]
        agents = np.asarray(agents, dtype=np.int64).reshape(-1)[keep]
        modalities = np.asarray(modalities, dtype=np.int64).reshape(-1)[keep]
        if class_probs is not None:
            class_probs = np.asarray(class_probs, dtype=np.float32)[keep]
        if kds_scores is not None:  # CHANGED (kds-orientation)
            kds_scores = np.asarray(kds_scores, dtype=np.float32)[keep]
    else:
        agents = np.asarray(agents, dtype=np.int64).reshape(-1)
        modalities = np.asarray(modalities, dtype=np.int64).reshape(-1)

    if len(boxes) == 0 or method in ("none", "identity"):
        return boxes, scores, classes, [np.array([i], dtype=np.int64) for i in range(len(boxes))]

    if method == "nms":
        out_boxes, out_scores, keep = non_maximum_suppression(boxes, scores, cfg.nms_iou_threshold)
        return out_boxes, out_scores, classes[keep], [np.array([int(i)], dtype=np.int64) for i in keep]

    if method == "psa":
        out_boxes, out_scores, keep = psa(boxes, scores)
        return out_boxes, out_scores, classes[keep], [np.array([int(i)], dtype=np.int64) for i in keep]

    if method in ("ms-psf", "mspsf"):
        if class_probs is None:
            num_classes = int(max(classes.max(initial=0) + 1, 1))
            class_probs = build_class_probs(classes, scores, num_classes)
        # CHANGED (kds-orientation)
        # Raw-label vehicle indicator for _orient_with_kds's class gate --
        # deliberately NOT derived from class_probs (which may itself be a
        # statistical synthesis via build_class_probs): the gate trusts the
        # reported label directly, see kinematic_checks.py's module docstring.
        vehicle_probs = (classes == _VEHICLE_CLASS_IDX).astype(np.float32)
        out_boxes, out_scores, out_class_probs, groups = mspsf(
            boxes,
            scores,
            class_probs,
            agents,
            modalities,
            cfg,
            agent_reliabilities=agent_reliabilities,
            kds_scores=kds_scores,  # CHANGED (kds-orientation)
            vehicle_probs=vehicle_probs,  # CHANGED (kds-orientation)
        )
        out_classes = np.argmax(out_class_probs, axis=1).astype(np.int64)
        out_scores = out_scores * np.max(out_class_probs, axis=1)
        return out_boxes, out_scores.astype(np.float32), out_classes, groups

    raise ValueError(f"Unsupported aggregation_method '{cfg.aggregation_method}'")
