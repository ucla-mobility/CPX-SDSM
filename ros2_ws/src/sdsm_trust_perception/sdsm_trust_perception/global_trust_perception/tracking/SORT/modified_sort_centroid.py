# Ported verbatim (pure Python, no ROS deps) from CPX-Mono's
# ros2/src/global_trust_tracker/global_trust_tracker/SORT/modified_SORT_centroid.py
# — see that repo for the full design writeup. No logic changed.
"""
    SORT: A Simple, Online and Realtime Tracker
    Copyright (C) 2016-2020 Alex Bewley alex@bewley.ai

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

Modified for cooperative perception (CPX):
  - Kalman state extended from [x,y,vx,vy] to [x,y,w,l,vx,vy] so width
    and length are filtered alongside position.
  - Height tracked separately via EMA (doesn't affect top-down matching).
  - Matching switched from Euclidean distance to 2D IoU on top-down
    axis-aligned boxes — more robust when objects are close together.
  - Velocity seeded from SDSM obj_speed/obj_heading on track init.
  - .confirmed flag preserved: set after min_hits consecutive detections.
"""
from __future__ import print_function

import numpy as np
from filterpy.kalman import KalmanFilter
from scipy.optimize import linear_sum_assignment


# --- IoU matching threshold (same role as iou_threshold in reference SORT) --
_IOU_THRESHOLD  = 0.3
# minimum gate floor used when falling back to distance (zero-dim detections)
_GATE_MIN_M     = 0.5
_GATE_TOLERANCE = 0.25
# EMA weight applied to each new height measurement
_HEIGHT_ALPHA   = 0.3


def linear_assignment(cost_matrix):
    x, y = linear_sum_assignment(cost_matrix)
    return np.array(list(zip(x, y)))


# --- Box helpers -------------------------------------------------------------

def _xy_wl_to_box(x: float, y: float, w: float, l: float) -> np.ndarray:
    """Centroid + (width, length) → top-down [x1, y1, x2, y2]."""
    return np.array([x - l / 2.0, y - w / 2.0, x + l / 2.0, y + w / 2.0])


def iou_batch(boxes_a: np.ndarray, boxes_b: np.ndarray) -> np.ndarray:
    """
    Pairwise 2D IoU between two sets of top-down axis-aligned boxes.

    boxes_a : (N, 4) [x1, y1, x2, y2]
    boxes_b : (M, 4) [x1, y1, x2, y2]
    Returns : (N, M) IoU matrix.
    """
    a = np.expand_dims(boxes_a, 1)   # (N, 1, 4)
    b = np.expand_dims(boxes_b, 0)   # (1, M, 4)

    xx1 = np.maximum(a[..., 0], b[..., 0])
    yy1 = np.maximum(a[..., 1], b[..., 1])
    xx2 = np.minimum(a[..., 2], b[..., 2])
    yy2 = np.minimum(a[..., 3], b[..., 3])

    inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
    area_a = (a[..., 2] - a[..., 0]) * (a[..., 3] - a[..., 1])
    area_b = (b[..., 2] - b[..., 0]) * (b[..., 3] - b[..., 1])
    union = area_a + area_b - inter

    return inter / np.maximum(union, 1e-6)


# --- Kalman tracker ----------------------------------------------------------

class KalmanCentroidTracker:
    """
    Tracks one object with a Kalman filter on [x, y, w, l, vx, vy].

    State  (dim_x=6): [x, y, w, l, vx, vy]
      x, y  — centroid in global frame (metres)
      w, l  — width and length (metres), modelled as slowly varying
      vx,vy — velocity (m/s), seeded from SDSM heading/speed on init

    Measurement (dim_z=4): [x, y, w, l]

    Height is tracked separately via an EMA — it changes very slowly
    and plays no role in the top-down IoU matching.
    """

    count = 0

    def __init__(self,
                 global_xy: np.ndarray,
                 dims: np.ndarray = np.zeros(3),
                 initial_vel: np.ndarray = np.zeros(2),
                 min_hits: int = 3):
        """
        global_xy   : (2,) [x, y] in metres
        dims        : (3,) [width, length, height] in metres
        initial_vel : (2,) [vx, vy] in m/s — seeded from SDSM heading/speed
        """
        self.min_hits  = min_hits
        self.confirmed = False
        self.height    = float(dims[2]) if len(dims) > 2 else 0.0

        # State: [x, y, w, l, vx, vy]
        self.kf = KalmanFilter(dim_x=6, dim_z=4)

        self.kf.F = np.array([
            [1, 0, 0, 0, 1, 0],   # x  += vx
            [0, 1, 0, 0, 0, 1],   # y  += vy
            [0, 0, 1, 0, 0, 0],   # w   (constant)
            [0, 0, 0, 1, 0, 0],   # l   (constant)
            [0, 0, 0, 0, 1, 0],   # vx  (constant)
            [0, 0, 0, 0, 0, 1],   # vy  (constant)
        ], dtype=float)

        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0],   # observe x
            [0, 1, 0, 0, 0, 0],   # observe y
            [0, 0, 1, 0, 0, 0],   # observe w
            [0, 0, 0, 1, 0, 0],   # observe l
        ], dtype=float)

        self.kf.R[2:, 2:]   *= 10.     # more measurement noise on w, l than on x, y
        self.kf.P[4:, 4:]   *= 10.     # modest uncertainty on seeded velocity
        self.kf.P           *= 10.
        self.kf.Q[4:, 4:]   *= 0.01    # low process noise on velocity
        self.kf.Q[2:4, 2:4] *= 0.001   # very low process noise on w, l

        self.kf.x[0] = global_xy[0]
        self.kf.x[1] = global_xy[1]
        self.kf.x[2] = float(dims[0]) if len(dims) > 0 else 0.0
        self.kf.x[3] = float(dims[1]) if len(dims) > 1 else 0.0
        self.kf.x[4] = initial_vel[0]
        self.kf.x[5] = initial_vel[1]

        self.time_since_update = 0
        self.id = KalmanCentroidTracker.count
        KalmanCentroidTracker.count += 1
        self.hits       = 0
        self.hit_streak = 0
        self.age        = 0

    def update(self, global_xy: np.ndarray, dims: np.ndarray = None):
        """
        Kalman correction step.

        global_xy : (2,) observed [x, y]
        dims      : (3,) observed [width, length, height]; if None the filter
                    uses its own current w, l estimate (position-only update).
        """
        self.time_since_update = 0
        self.hits += 1
        self.hit_streak += 1
        if self.hit_streak >= self.min_hits:
            self.confirmed = True

        if dims is not None and len(dims) >= 2:
            w = float(dims[0])
            l = float(dims[1])
            if len(dims) > 2:
                self.height = ((1 - _HEIGHT_ALPHA) * self.height
                               + _HEIGHT_ALPHA * float(dims[2]))
        else:
            w = float(self.kf.x[2])
            l = float(self.kf.x[3])

        z = np.array([[global_xy[0]], [global_xy[1]], [w], [l]])
        self.kf.update(z)

    def predict(self) -> np.ndarray:
        """Advance Kalman state; return predicted [x, y]."""
        self.kf.predict()
        self.age += 1
        if self.time_since_update > 0:
            self.hit_streak = 0
        self.time_since_update += 1
        return self.kf.x[:2].flatten()

    def get_state(self) -> np.ndarray:
        """Return current [x, y] centroid estimate."""
        return self.kf.x[:2].flatten()

    def get_dims(self) -> tuple[float, float, float]:
        """Return current (width, length, height) estimate in metres."""
        return float(self.kf.x[2]), float(self.kf.x[3]), self.height

    def get_box(self) -> np.ndarray:
        """Return top-down [x1, y1, x2, y2] from the current state estimate."""
        x, y = float(self.kf.x[0]), float(self.kf.x[1])
        w, l = float(self.kf.x[2]), float(self.kf.x[3])
        return _xy_wl_to_box(x, y, w, l)

    def expected_gate(self, tolerance: float = _GATE_TOLERANCE,
                      min_dist: float = _GATE_MIN_M) -> float:
        """Euclidean gate (metres) — used as fallback when dims are zero."""
        speed = float(np.linalg.norm(self.kf.x[4:6]))
        return max(min_dist, (1.0 + tolerance) * speed)


# --- Matching ----------------------------------------------------------------

def associate_detections_to_trackers(
        det_boxes: np.ndarray,
        trk_boxes: np.ndarray,
        trk_trackers: list,
        iou_threshold: float = _IOU_THRESHOLD):
    """
    Assign detections to tracked objects.

    Primary: 2D IoU on top-down axis-aligned boxes. Used when at least one
    box pair overlaps (i.e. dims are available and non-zero).

    Fallback: Euclidean centroid distance with per-track dynamic gates.
    Used when all IoU values are zero — this happens when dims are None/zero
    (point-boxes). Preserves the original distance-matching behaviour so
    callers that don't supply dims still work correctly.

    det_boxes    : (N, 4) [x1,y1,x2,y2]
    trk_boxes    : (M, 4) [x1,y1,x2,y2]
    trk_trackers : list of KalmanCentroidTracker, length M

    Returns (matches, unmatched_detections, unmatched_trackers).
    """
    if len(trk_boxes) == 0:
        return (np.empty((0, 2), dtype=int),
                np.arange(len(det_boxes)),
                np.empty((0,), dtype=int))

    iou_matrix = iou_batch(det_boxes, trk_boxes)   # (N, M)
    use_iou = iou_matrix.max() > 0                 # any real overlap → use IoU

    if use_iou:
        # IoU matching (dims available)
        if min(iou_matrix.shape) > 0:
            a = (iou_matrix > iou_threshold).astype(np.int32)
            if a.sum(1).max() == 1 and a.sum(0).max() == 1:
                matched_indices = np.stack(np.where(a), axis=1)
            else:
                matched_indices = linear_assignment(-iou_matrix)
        else:
            matched_indices = np.empty((0, 2), dtype=int)

        unmatched_dets = [d for d in range(len(det_boxes))
                          if d not in matched_indices[:, 0]]
        unmatched_trks = [t for t in range(len(trk_boxes))
                          if t not in matched_indices[:, 1]]

        matches = []
        for m in matched_indices:
            if iou_matrix[m[0], m[1]] < iou_threshold:
                unmatched_dets.append(int(m[0]))
                unmatched_trks.append(int(m[1]))
            else:
                matches.append(m.reshape(1, 2))

    else:
        # Distance fallback (dims unavailable → point-boxes → IoU always 0)
        det_centroids = (det_boxes[:, :2] + det_boxes[:, 2:4]) / 2   # (N, 2)
        trk_centroids = (trk_boxes[:, :2] + trk_boxes[:, 2:4]) / 2   # (M, 2)
        dist_matrix = np.linalg.norm(
            det_centroids[:, None, :] - trk_centroids[None, :, :], axis=2
        )
        gates = [t.expected_gate() for t in trk_trackers]

        matched_indices = (linear_assignment(dist_matrix)
                           if min(dist_matrix.shape) > 0
                           else np.empty((0, 2), dtype=int))

        unmatched_dets = [d for d in range(len(det_boxes))
                          if d not in matched_indices[:, 0]]
        unmatched_trks = [t for t in range(len(trk_boxes))
                          if t not in matched_indices[:, 1]]

        matches = []
        for m in matched_indices:
            if dist_matrix[m[0], m[1]] > gates[m[1]]:
                unmatched_dets.append(int(m[0]))
                unmatched_trks.append(int(m[1]))
            else:
                matches.append(m.reshape(1, 2))

    matches = (np.concatenate(matches, axis=0) if matches
               else np.empty((0, 2), dtype=int))
    return matches, np.array(unmatched_dets), np.array(unmatched_trks)


# --- Modified SORT -----------------------------------------------------------

class Sort:
    """
    Modified SORT for cooperative perception.

    update() accepts per-detection dims (width, length, height) alongside
    positions and velocities. When dims are provided, matching uses 2D IoU
    on top-down bounding boxes; the Kalman filter tracks w and l as
    slowly-varying state, and height is smoothed via EMA.

    Returns a list parallel to input detections; each element is a
    KalmanCentroidTracker with .confirmed, .get_dims(), and .get_box().
    """

    def __init__(self, max_age: int = 3, min_hits: int = 3,
                 iou_threshold: float = _IOU_THRESHOLD):
        self.max_age       = max_age
        self.min_hits      = min_hits
        self.iou_threshold = iou_threshold
        self.trackers: list[KalmanCentroidTracker] = []
        self.frame_count   = 0

    def update(self,
               global_xy: np.ndarray,
               velocities: np.ndarray = None,
               dims: np.ndarray = None) -> list:
        """
        Advance one frame; return one KalmanCentroidTracker per input detection.

        global_xy  : (N, 2) already-projected global [x, y] positions.
                     Pass np.empty((0, 2)) for frames with no detections.
        velocities : (N, 2) [vx, vy] in m/s — seeds new track velocity.
                     Pass None to default to zeros.
        dims       : (N, 3) [width, length, height] in metres.
                     Pass None when dimensions are unavailable; matching
                     falls back to distance gating.

        Returns list parallel to global_xy; each element has .confirmed,
        .get_dims() → (w, l, h), and .get_box() → [x1,y1,x2,y2].
        """
        self.frame_count += 1
        n_det = len(global_xy)
        assigned: list = [None] * n_det

        # Predict all existing tracks; collect predicted boxes
        predicted_boxes = []
        to_del = []
        for t, trk in enumerate(self.trackers):
            p = trk.predict()
            if np.any(np.isnan(p)):
                to_del.append(t)
            else:
                predicted_boxes.append(trk.get_box())
        for t in reversed(to_del):
            self.trackers.pop(t)

        trk_boxes = (np.array(predicted_boxes)
                     if predicted_boxes else np.empty((0, 4)))

        # Build detection boxes from current detections + dims
        if n_det > 0 and dims is not None:
            det_boxes = np.array([
                _xy_wl_to_box(global_xy[i, 0], global_xy[i, 1],
                              dims[i, 0], dims[i, 1])
                for i in range(n_det)
            ])
        elif n_det > 0:
            # No dims: use zero-area point boxes — IoU will be 0 everywhere,
            # so all detections become unmatched and spawn new tracks.
            det_boxes = np.array([
                _xy_wl_to_box(global_xy[i, 0], global_xy[i, 1], 0.0, 0.0)
                for i in range(n_det)
            ])
        else:
            det_boxes = np.empty((0, 4))

        matched_tracks: set = set()

        if len(self.trackers) and n_det:
            matched, unmatched_dets, _ = associate_detections_to_trackers(
                det_boxes, trk_boxes, self.trackers, self.iou_threshold
            )
            for m in matched:
                d_dims = dims[m[0]] if dims is not None else None
                self.trackers[m[1]].update(global_xy[m[0]], d_dims)
                assigned[m[0]] = self.trackers[m[1]]
                matched_tracks.add(int(m[1]))
        else:
            unmatched_dets = list(range(n_det))

        for ti in range(len(self.trackers)):
            if ti not in matched_tracks:
                self.trackers[ti].hit_streak = 0

        # Spawn new tracks for unmatched detections
        for dk in unmatched_dets:
            vel  = velocities[dk] if velocities is not None else np.zeros(2)
            d    = dims[dk]       if dims is not None       else np.zeros(3)
            trk  = KalmanCentroidTracker(global_xy[dk], d, vel, self.min_hits)
            self.trackers.append(trk)
            assigned[dk] = trk

        # Prune stale tracks
        self.trackers = [t for t in self.trackers
                         if t.time_since_update <= self.max_age]

        return assigned
