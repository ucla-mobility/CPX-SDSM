"""Pipeline <-> MS-PSF boundary.

This is the ONLY module that converts trust-pipeline data (global positions,
dims, headings, per-object scores, per-agent reputations) into the fusion
engine's rotated-BEV representation and back. Nothing outside this subpackage
constructs a ``(4, 2)`` corner array or a ``FusionConfig``; the orchestrator
and the node deal only in the plain values defined here.

One ``fuse()`` call drives both roles of the two-phase design:

  * phase 1 (judging): caller passes ego + every agent with reliabilities =
    current reputation (ego = 1). Reads ``clusters`` (consensus grouping);
    what that grouping is worth as corroboration is trust policy and lives in
    consistency.corroboration_support, not here.
  * phase 2 (output/visualization): caller passes only the admitted agents.
    Reads the fused geometry + ``contributors``.

Both phases use the same aggregation config (``FUSION_CONFIG``); what differs
between them is the input agent set and the reliabilities, not the algorithm.

Coordinate conventions:
  * positions are global (x, y, z) in metres; dims are (w, l, h) in metres
    (get_dims_of's native order); headings are J2735 degrees (0 = North,
    clockwise, 28800 = unavailable).
  * MS-PSF works in bird's-eye view: each box is 4 rotated corners in the
    (x, y) ground plane. z-centre and height are carried alongside and fused
    with a score*reliability weighted mean (display only — the trust pipeline
    never uses z), so the 3D visualization still gets full boxes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .display_derivation import (
    EMPTY_BOUNDS,
    contributors_of,
    fused_z_and_height,
    scene_bounds,
)
from .fusion import FusionConfig, fuse_detections
from .geometry import corners_from_pose

# --- Constants -------------------------------------------------------------

# J3224 equipment types 0..3 map directly to MS-PSF modality ids; feeds the
# config's mspsf_total_modalities so the modality-entropy term is normalised
# against the real number of source types even while every sender is an OBU.
EQUIPMENT_TYPE_COUNT = 4

# Floor for any BEV box dimension (m): a zero-size box has no polygon and no
# IoU. Succeeds matching._MIN_HALF_DIM * 2 from the retired WBF path.
MIN_BOX_DIM_M = 0.3

# J2735 heading sentinel: 28800 * 0.0125 deg = 360 -> "unavailable". Treated
# as heading 0 (axis-aligned) rather than a real bearing.
_HEADING_UNAVAILABLE_DEG = 360.0

# Single shared aggregation config for both fusion phases. mspsf_total_modalities
# is pinned to the real equipment-type count; every other field keeps the
# vendored MS-PSF defaults.
FUSION_CONFIG = FusionConfig(
    aggregation_method="mspsf",
    score_threshold=0.0,  # 0 => no pre-filter, so group indices match our input order
    mspsf_total_modalities=EQUIPMENT_TYPE_COUNT,
)


# --- Inputs / outputs ------------------------------------------------------

@dataclass
class StreamInput:
    """One agent's (or ego's) detections plus its trust weight for this frame.

    Missing per-detection lists (or ones whose length disagrees with
    ``positions``) fall back to per-detection defaults, matching the
    defensive style of the rest of the pipeline.
    """

    key: object                       # stream identity: ego sentinel or agent_id
    positions: list                   # (x, y, z) metres
    dims: Optional[list] = None       # (w, l, h) metres; None -> MIN_BOX_DIM_M cube
    headings: Optional[list] = None   # J2735 degrees; None -> 0 (axis-aligned)
    scores: Optional[list] = None     # local certainty [0,1]; None -> 1.0
    labels: Optional[list] = None     # obj_type class ids; None -> 0
    modality: int = 0                 # equipment_type -> MS-PSF modality
    reliability: float = 1.0          # reputation weight; ego = 1.0
    kds: Optional[list] = None        # per-detection Kinematic-Dynamic
                                       # Consistency Score (eq 2.8), [0,1];
                                       # None -> 1.0 (fully consistent)


@dataclass
class FusionResult:
    """Everything derived from one fusion pass.

    Phase 1 reads ``clusters`` (and hands it to consistency.corroboration_support
    to value that membership); phase 2 reads the fused geometry and
    ``contributors``. ``bounds`` is the padded scene extent for the
    visualization axes.
    """

    clusters: list = field(default_factory=list)          # [{stream_key: det_idx}]
    fused_boxes: list = field(default_factory=list)        # [(4,2) BEV corners]
    fused_centers_z: list = field(default_factory=list)    # z-centre (m) per fused box
    fused_heights: list = field(default_factory=list)      # height (m) per fused box
    fused_scores: list = field(default_factory=list)       # fused certainty [0,1]
    fused_labels: list = field(default_factory=list)       # fused class id
    contributors: list = field(default_factory=list)       # [[stream_key,...]] ego-first
    bounds: tuple = EMPTY_BOUNDS


# --- Public API ------------------------------------------------------------

def fuse(streams: list, ego_key: object, bounds_padding: float = 2.0) -> FusionResult:
    """Run one MS-PSF pass over the given streams.

    streams  : list of StreamInput — ego included for phase 1, excluded (or
               only the admitted agents) for phase 2. Empty streams are ignored.
    ego_key  : the key identifying ego's stream; contributors list it first.

    Returns a FusionResult; on no detections, an empty result with fallback
    bounds so callers can render an empty frame without special-casing.
    """
    used = [s for s in streams if s.positions]
    bounds = scene_bounds(used, bounds_padding)
    if not used:
        return FusionResult(bounds=bounds)

    # Stable stream order: ego first (so contributor lists lead with ego),
    # then the rest by key. Gives each stream a contiguous MS-PSF agent index.
    order = sorted(used, key=lambda s: (s.key != ego_key, _sort_key(s.key)))
    agent_index = {s.key: i for i, s in enumerate(order)}
    reliabilities = np.array([max(float(s.reliability), 0.0) for s in order], dtype=np.float32)

    boxes, scores, labels, agents, modalities, kds = [], [], [], [], [], []
    owner: list[tuple] = []      # flat_idx -> (stream_key, det_idx)
    centers_z, heights = [], []  # flat_idx -> z-centre, height (carried, not fused in BEV)

    for s in order:
        n = len(s.positions)
        dims = _rows(s.dims, n, (0.0, 0.0, 0.0))
        hdgs = _rows(s.headings, n, 0.0)
        scrs = _rows(s.scores, n, 1.0)
        lbls = _rows(s.labels, n, 0)
        kdss = _rows(s.kds, n, 1.0)
        for j in range(n):
            cx, cy, cz = s.positions[j][0], s.positions[j][1], s.positions[j][2]
            # StreamInput.dims is (WIDTH, LENGTH, height) -- get_dims_of's
            # native order -- while corners_from_pose takes length first and
            # lays it along the heading. Reading them in declaration order
            # builds every box 90 degrees off: a 2.0 x 4.5 m car comes out
            # 2.0 m along its heading and 4.5 m across it.
            w      = max(float(dims[j][0]), MIN_BOX_DIM_M)
            length = max(float(dims[j][1]), MIN_BOX_DIM_M)
            h = max(float(dims[j][2]), MIN_BOX_DIM_M)
            boxes.append(corners_from_pose(cx, cy, length, w, _yaw_of(hdgs[j])))
            scores.append(float(scrs[j]))
            labels.append(int(lbls[j]))
            agents.append(agent_index[s.key])
            modalities.append(int(s.modality))
            kds.append(float(kdss[j]))
            owner.append((s.key, j))
            centers_z.append(float(cz))
            heights.append(h)

    boxes_arr = np.asarray(boxes, dtype=np.float32)
    scores_arr = np.asarray(scores, dtype=np.float32)

    _fboxes, fscores, flabels, groups = fuse_detections(
        boxes_arr,
        scores_arr,
        np.asarray(labels, dtype=np.int64),
        np.asarray(agents, dtype=np.int64),
        np.asarray(modalities, dtype=np.int64),
        FUSION_CONFIG,
        agent_reliabilities=reliabilities,
        kds_scores=np.asarray(kds, dtype=np.float32),
    )

    result = FusionResult(bounds=bounds)
    reliability_of = {s.key: float(s.reliability) for s in order}

    for gi, group in enumerate(groups):
        members = [owner[int(f)] for f in group]
        keys_in_group = {k for k, _ in members}

        # Cluster: at most one detection per stream (highest-scoring wins), so
        # the orchestrator's matched/ego_only/other_only derivation is unchanged.
        cluster: dict = {}
        best: dict = {}
        for f in group:
            f = int(f)
            k, det = owner[f]
            if k not in best or scores_arr[f] > best[k]:
                cluster[k] = det
                best[k] = float(scores_arr[f])
        result.clusters.append(cluster)

        # Display-side derivation (z/height, contributors): BEV fusion yields
        # neither, and the trust pipeline needs neither — see display_derivation.
        cz, h = fused_z_and_height(
            group, owner, scores_arr, reliability_of, centers_z, heights)
        result.fused_centers_z.append(cz)
        result.fused_heights.append(h)
        result.contributors.append(
            contributors_of(keys_in_group, [s.key for s in order]))

        result.fused_boxes.append(np.asarray(_fboxes[gi], dtype=np.float32))
        result.fused_scores.append(float(fscores[gi]))
        result.fused_labels.append(int(flabels[gi]))

    return result


# --- Internal helpers ------------------------------------------------------

def _yaw_of(heading_deg: float) -> float:
    """J2735 heading (deg, 0=North CW) -> math yaw (rad, 0=+x East CCW)."""
    if heading_deg is None or float(heading_deg) >= _HEADING_UNAVAILABLE_DEG:
        return 0.0
    return math.radians(90.0 - float(heading_deg))


def _rows(seq, n: int, default):
    """Return a length-n list, substituting default when seq is absent or ragged."""
    if seq is None or len(seq) != n:
        return [default] * n
    return list(seq)


def _sort_key(key):
    """Order streams deterministically even with mixed key types."""
    return (0, key) if isinstance(key, (int, float)) else (1, str(key))
