"""Geometry utilities for rotated BEV boxes.

The ROS node uses the same runtime box representation as the original
MMCooperFuse code: ``(N, 4, 2)`` arrays of BEV corner points.
"""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np
# ========================== CHANGED (vectorized) ===========================
# Top-level `shapely` (2.x array API) alongside the existing per-object
# `shapely.geometry` import: compute_self_iou_mat below needs the ufunc-style
# entry points (shapely.polygons/area/intersection) that operate on whole
# numpy arrays of geometries. The object API is still imported and still used
# by to_polygon/_safe_iou, which nms_rotated calls one pair at a time.
import shapely
# ======================== END CHANGED (vectorized) =========================
from scipy.spatial.distance import cdist
from shapely.geometry import Polygon
from shapely.errors import GEOSException


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Return a z-axis quaternion for planar yaw."""
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def corners_from_pose(
    center_x: float,
    center_y: float,
    length: float,
    width: float,
    yaw: float,
) -> np.ndarray:
    """Build four BEV corners from center, dimensions, and yaw."""
    dx = 0.5 * max(float(length), 1e-6)
    dy = 0.5 * max(float(width), 1e-6)
    local = np.array(
        [[dx, dy], [dx, -dy], [-dx, -dy], [-dx, dy]],
        dtype=np.float32,
    )
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]], dtype=np.float32)
    return local @ rot.T + np.array([center_x, center_y], dtype=np.float32)


def pose_from_corners(corners: np.ndarray) -> tuple[float, float, float, float, float]:
    """Approximate center, length, width, and yaw from four BEV corners."""
    pts = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    center = pts.mean(axis=0)
    edge01 = pts[0] - pts[1]
    edge12 = pts[1] - pts[2]
    width = float(np.linalg.norm(edge01))
    length = float(np.linalg.norm(edge12))
    heading = pts[0] - pts[3]
    yaw = math.atan2(float(heading[1]), float(heading[0]))
    return float(center[0]), float(center[1]), length, width, yaw


def to_polygon(box: np.ndarray) -> Polygon:
    """Convert a ``(4, 2)`` BEV box into a Shapely polygon."""
    pts = np.asarray(box, dtype=np.float32).reshape(4, 2)
    return Polygon([(float(pts[i, 0]), float(pts[i, 1])) for i in range(4)])


def _safe_iou(poly_a: Polygon, poly_b: Polygon) -> float:
    try:
        union = poly_a.union(poly_b).area
        if union <= 0.0:
            return 0.0
        return float(poly_a.intersection(poly_b).area / union)
    except GEOSException:
        return 0.0


def compute_iou(box: Polygon, boxes: Iterable[Polygon]) -> np.ndarray:
    """Compute IoU between one polygon and a sequence of polygons."""
    return np.array([_safe_iou(box, b) for b in boxes], dtype=np.float32)


def compute_self_iou_mat(boxes: np.ndarray, dist_threshold: float = 10.0) -> np.ndarray:
    """Compute a distance-pruned pairwise IoU matrix for rotated boxes."""
    boxes = np.asarray(boxes, dtype=np.float32).reshape(-1, 4, 2)
    if len(boxes) == 0:
        return np.zeros((0, 0), dtype=np.float32)

    centers = boxes.mean(axis=1)
    dist_mat = cdist(centers, centers)
    iou_mat = np.zeros_like(dist_mat, dtype=np.float32)
    np.fill_diagonal(iou_mat, 1.0)

    # ========================= CHANGED (vectorized) =========================
    # WAS:
    #   polygons = [to_polygon(boxes[i]) for i in range(len(boxes))]
    #
    #   for i in range(len(boxes)):
    #       for j in range(i + 1, len(boxes)):
    #           if dist_mat[i, j] < dist_threshold:
    #               iou = _safe_iou(polygons[i], polygons[j])
    #               iou_mat[i, j] = iou
    #               iou_mat[j, i] = iou
    #
    # WHY IT IS FASTER: the old form paid a Python-level loop iteration and a
    # per-object Shapely call for every one of the n(n-1)/2 pairs, even though
    # the distance gate rejects almost all of them. Here the gate is a single
    # boolean mask over triu_indices, and the surviving pairs cross into GEOS
    # once as arrays rather than once per pair. Polygon construction is also
    # batched -- shapely.polygons() takes the whole (N, 4, 2) float32 array and
    # auto-closes each ring, replacing N Python Polygon() constructions.
    #
    # WHY IT IS THE SAME: same coordinates (boxes is already float32, and GEOS
    # widens to float64 identically whether it comes from the array or from
    # to_polygon's per-corner float() casts), same gate (strict <), same
    # symmetric write, diagonal still pinned to 1.0. Intersection area is the
    # identical GEOS op. Union is inclusion-exclusion (area_i + area_j - inter)
    # instead of area(poly_a.union(poly_b)) -- mathematically exact for valid
    # polygons, so the two agree to float64 round-off and hence agree after the
    # float32 store, but this is NOT proven bit-identical the way the fusion.py
    # blocks are, and there is no _ref_ copy of this body in
    # test/test_fusion_equivalence.py pinning it.
    #
    # ALSO DROPPED: _safe_iou's try/except GEOSException. Reachable only for a
    # self-intersecting (bow-tie) quad, which corners_from_pose cannot produce;
    # the union <= 0 guard below still covers degenerate zero-area boxes.
    ii, jj = np.triu_indices(len(boxes), 1)
    close = dist_mat[ii, jj] < dist_threshold
    ii, jj = ii[close], jj[close]
    if len(ii):
        polygons = shapely.polygons(boxes)
        areas = shapely.area(polygons)
        inter = shapely.area(shapely.intersection(polygons[ii], polygons[jj]))
        union = areas[ii] + areas[jj] - inter
        iou = np.zeros(len(ii), dtype=np.float32)
        valid = union > 0.0
        iou[valid] = (inter[valid] / union[valid]).astype(np.float32)
        iou_mat[ii, jj] = iou
        iou_mat[jj, ii] = iou
    # ======================= END CHANGED (vectorized) =======================
    return iou_mat


def rotated_weighted_boxes_fusion(
    boxes: np.ndarray,
    weights: np.ndarray,
    eps: float = 1e-8,
) -> np.ndarray:
    """Weighted corner averaging for rotated BEV boxes."""
    boxes = np.asarray(boxes, dtype=np.float32).reshape(-1, 4, 2)
    if len(boxes) == 1:
        return boxes[0].copy()
    w = np.maximum(np.asarray(weights, dtype=np.float32).reshape(-1), 0.0)
    w_norm = w / (float(np.sum(w)) + eps)
    return np.tensordot(w_norm, boxes, axes=1).astype(np.float32)
