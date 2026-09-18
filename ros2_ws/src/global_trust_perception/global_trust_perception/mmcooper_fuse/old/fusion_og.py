# mypy: ignore-errors
"""Pre-optimization ("old") copies of the two MS-PSF hot paths, shared by both
speedup views so there is a single source of truth for the "before" arm.

Holds the old bodies of exactly the two `# ==== CHANGED (vectorized) ====`
speed rewrites, and nothing else:

  * ``compute_self_iou_mat_old`` -- the per-pair Shapely loop that the batched
    GEOS call in ``geometry.compute_self_iou_mat`` replaced.
  * ``mspsf_old`` -- a copy of ``fusion.mspsf`` with only its affinity_boost /
    kappa double-loops restored. Everything else (seed selection, the
    kds-orientation override, the modality-entropy term) is unchanged, so with
    ``kds_scores=None`` this is the pre-vectorization fusion and with kds passed
    through it keeps the feature -- both comparisons isolate the *speed* rewrites.

Consumers:
  * ``scripts/benchmark_stages.py --fusion reference`` monkeypatches
    ``fusion.mspsf = mspsf_old`` to time the whole pipeline without the rewrites.
  * ``scripts/make_fusion_speedup_viz.py`` calls ``mspsf_old`` directly (with
    ``kds_scores=None``) as the "before" bar of the fusion-stage figure.

This is the current code with the vectorized blocks reverted, not a checkout of
the original 6b05baaa CooperFuse -- so "before" here means "today's fusion minus
the vectorization", which isolates exactly the rewrite rather than folding in
every unrelated change since. Every helper that is not part of a reverted block
is imported from the live modules; only the two reverted regions live here. If
``fusion.mspsf`` changes outside the vectorized block, regenerate this from it.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.distance import cdist

from ..fusion import FusionConfig, _orient_with_kds, rotated_weighted_boxes_fusion
from ..geometry import _safe_iou, to_polygon


def compute_self_iou_mat_old(boxes: np.ndarray, dist_threshold: float = 10.0) -> np.ndarray:
    """Pre-rewrite ``geometry.compute_self_iou_mat`` (per-pair Shapely loop)."""
    boxes = np.asarray(boxes, dtype=np.float32).reshape(-1, 4, 2)
    if len(boxes) == 0:
        return np.zeros((0, 0), dtype=np.float32)

    centers = boxes.mean(axis=1)
    dist_mat = cdist(centers, centers)
    iou_mat = np.zeros_like(dist_mat, dtype=np.float32)
    np.fill_diagonal(iou_mat, 1.0)

    polygons = [to_polygon(boxes[i]) for i in range(len(boxes))]

    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            if dist_mat[i, j] < dist_threshold:
                iou = _safe_iou(polygons[i], polygons[j])
                iou_mat[i, j] = iou
                iou_mat[j, i] = iou
    return iou_mat


def mspsf_old(
    boxes: np.ndarray,
    scores: np.ndarray,
    class_probs: np.ndarray,
    agents: np.ndarray,
    modalities: np.ndarray,
    cfg: FusionConfig,
    agent_reliabilities: np.ndarray | None = None,
    kds_scores: np.ndarray | None = None,
    vehicle_probs: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[np.ndarray]]:
    """Copy of ``fusion.mspsf`` with only the affinity_boost / kappa double-loops
    restored (and the IoU matrix taken from ``compute_self_iou_mat_old``). See
    the module docstring for scope."""
    boxes = np.asarray(boxes, dtype=np.float32).reshape(-1, 4, 2)
    scores = np.asarray(scores, dtype=np.float32).reshape(-1)
    class_probs = np.asarray(class_probs, dtype=np.float32).reshape(len(boxes), -1)
    agents = np.asarray(agents, dtype=np.int64).reshape(-1)
    modalities = np.asarray(modalities, dtype=np.int64).reshape(-1)
    n = len(boxes)
    if n == 0:
        return boxes, scores, class_probs, []

    iou = compute_self_iou_mat_old(boxes, dist_threshold=cfg.mspsf_distance_threshold)
    affinity_boost = np.ones((n, n), dtype=np.float32)
    for i in range(n):
        for j in range(i + 1, n):
            bonus = 0.0
            if agents[i] != agents[j]:
                bonus += cfg.mspsf_gamma_agent
            if modalities[i] != modalities[j]:
                bonus += cfg.mspsf_gamma_modality
            affinity_boost[i, j] = affinity_boost[j, i] = 1.0 + bonus
    affinity = iou * affinity_boost

    kappa = np.ones((n, n), dtype=np.float32)
    for i in range(n):
        for j in range(i + 1, n):
            kappa[i, j] = kappa[j, i] = np.minimum(class_probs[i], class_probs[j]).sum()

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
            if kds is not None and len(group_arr) > 1:
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
