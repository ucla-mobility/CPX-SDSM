"""Display-side derivation from a fusion pass.

MS-PSF works in bird's-eye view: it groups rotated (x, y) boxes and knows
nothing about z, about which stream is ego, or about the extent of the scene.
None of that is needed to JUDGE agents — phase 1 reads cluster membership and
stops there. It is needed to DRAW them, so everything the fused output needs to
become a picture lives here rather than in the pipeline<->fusion boundary:

  * z-centre / height  : carried alongside the BEV boxes and averaged per group,
                         since the trust pipeline never uses z but a 3D panel
                         cannot draw a box without it,
  * contributors       : which streams fed each fused box, ego first, for the
                         legend and the per-combination colouring,
  * scene bounds       : the padded extent the visualization axes are set to,
  * fused_pose()       : (4, 2) corners + z/height -> a drawable pose.

Kept inside mmcooper_fuse so the ``(4, 2)`` corner array never escapes the
subpackage: callers hand one in and get plain floats back. The consumers
(visualization.py, scripts/mspsf_fusion_stub.py) name no fusion algorithm.
"""

from __future__ import annotations

import numpy as np

from .geometry import pose_from_corners

# Fallback extent when nothing was detected, so an empty frame still renders
# with sane axes instead of a degenerate zero-size box.
EMPTY_BOUNDS = (-1.0, 1.0, -1.0, 1.0, -1.0, 1.0)


def scene_bounds(streams: list, padding: float) -> tuple:
    """(x_lo,x_hi,y_lo,y_hi,z_lo,z_hi) enclosing every detection, plus padding."""
    pts = [p for s in streams for p in s.positions]
    if not pts:
        return EMPTY_BOUNDS
    xs, ys, zs = zip(*[(p[0], p[1], p[2]) for p in pts])
    return (
        min(xs) - padding, max(xs) + padding,
        min(ys) - padding, max(ys) + padding,
        min(zs) - padding, max(zs) + padding,
    )


def fused_z_and_height(group, owner, scores_arr, reliability_of,
                       centers_z: list, heights: list) -> tuple:
    """Score*reliability weighted mean z-centre and height over one group.

    Falls back to a plain mean when every weight vanishes (an all-zero-
    reputation group would otherwise divide by zero), so a group always yields
    a drawable box.
    """
    w = np.array([
        scores_arr[int(f)] * reliability_of.get(owner[int(f)][0], 0.0)
        for f in group
    ], dtype=np.float64)
    if w.sum() <= 0.0:
        w = np.ones(len(group), dtype=np.float64)
    w /= w.sum()
    gz = [centers_z[int(f)] for f in group]
    gh = [heights[int(f)] for f in group]
    return float(np.dot(w, gz)), float(np.dot(w, gh))


def contributors_of(keys_in_group: set, stream_order: list) -> list:
    """Streams that fed one fused box, in the caller's stream order (ego first)."""
    return [k for k in stream_order if k in keys_in_group]


def fused_pose(box: np.ndarray, center_z: float, height: float) -> tuple:
    """(4,2) BEV corners + z/h -> (cx, cy, cz, l, w, h, yaw) for drawing."""
    cx, cy, length, width, yaw = pose_from_corners(box)
    return cx, cy, float(center_z), length, width, float(height), yaw
