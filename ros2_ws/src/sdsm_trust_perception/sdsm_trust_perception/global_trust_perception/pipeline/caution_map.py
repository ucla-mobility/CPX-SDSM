"""
Caution layer for the ``strict_unverified`` admission mode.

The trust engine sorts every reported object into three groups:

  confirmed   trusted sender AND corroborated       -> ordinary map object
  unverified  trusted sender, nobody confirms it,   -> CAUTION marker (this module)
              nobody contradicts it
  rejected    untrusted sender or contradicted      -> dropped

An unverified object is often a real vehicle or obstacle only one sensor can see (hidden
behind a truck, around a bend). Discarding it is unsafe, trusting it as a normal car is
unsafe, so the vehicle treats it as a *potential critical vehicle*: it is put on the map with
a larger keep-out distance, and the layer reports whether the ego's own path passes through
that keep-out zone soon. A fake that lands here costs some extra caution, never a trusted
phantom car.

This module is pure (numpy only, no ROS) so trust_node.py and analysis/replay_trust_verdicts.py
share one implementation.
"""

from __future__ import annotations

from typing import NamedTuple, Optional

import numpy as np

# Footprint of a conflict, in the ego's direction of travel: two cars overlap when their centres
# are closer than half the sum of their sizes (1.8 x 4.5 m each), so 4.5 m along and 1.8 m across.
COLLISION_HALF_LEN_M = 4.5
COLLISION_HALF_WID_M = 1.8
# Extra keep-out around a CAUTION marker (a vehicle nobody has confirmed): mostly along the road,
# i.e. extra following distance, and a little sideways.
CAUTION_MARGIN_M = 3.0
CAUTION_LAT_MARGIN_M = 0.5
# Look-ahead for the conflict test and its time step (seconds).
CRITICAL_HORIZON_S = 4.0
CRITICAL_STEP_S = 0.25
# Markers farther than this from the ego do not raise the "caution" advisory (metres), and it is
# also the distance at which a marker's proximity weight falls to zero.
ADVISORY_RANGE_M = 60.0
# A marker on the ego's predicted path is only CRITICAL once its criticality reaches this.
# criticality = proximity x urgency, both 0..1 (see CautionMap.update).
CRITICAL_SCORE_MIN = 0.2
# Neighbours report the ego's own car back as an object; anything this close to the ego is the ego.
EGO_SELF_RADIUS_M = 2.0
# A caution marker this close to a confirmed object, or to something the ego itself senses,
# is redundant: the space is already covered by a trusted object (metres).
DUPLICATE_RADIUS_M = 6.0
# A marker stays on the map this long after its last unverified sighting, so radio loss or
# a one-frame flicker does not make a possible hazard vanish (seconds).
CAUTION_TTL_S = 2.0

ADVISORY_CLEAR = 'clear'        # no caution markers near the ego
ADVISORY_CAUTION = 'caution'    # a caution marker is within ADVISORY_RANGE_M, none on the ego's path
ADVISORY_CRITICAL = 'critical'  # the ego's path enters a caution keep-out box soon


class Sightings(NamedTuple):
    """N objects reported this frame, already dead-reckoned to the frame's instant."""

    sender: np.ndarray   # (N,) int   sender node id
    oid: np.ndarray      # (N,) int   object id as reported
    xy: np.ndarray       # (N, 2)     position, metres
    v: np.ndarray        # (N, 2)     velocity, m/s

    @staticmethod
    def empty() -> 'Sightings':
        return Sightings(np.empty(0, dtype=int), np.empty(0, dtype=int),
                         np.empty((0, 2)), np.empty((0, 2)))

    def __len__(self) -> int:
        return len(self.sender)


class CautionMarker(NamedTuple):
    sender: int
    object_id: int
    x: float
    y: float
    vx: float
    vy: float
    keepout_m: float     # along-road keep-out: footprint half-length + caution margin
    critical: bool       # the ego's path enters the keep-out box within the horizon
    ttc_s: float         # time the ego first enters it (0 if not critical)
    min_dist_m: float    # closest centre-to-centre distance within the horizon
    dist_m: float        # distance from the ego now
    criticality: float   # 0..1: proximity x urgency; rises steeply as the marker nears the ego


class FrameMap(NamedTuple):
    markers: tuple             # CautionMarker for every live caution marker
    n_critical_confirmed: int  # confirmed objects whose path conflicts with the ego's
    advisory: str


def conflicts(ego_xy, ego_v, xy: np.ndarray, v: np.ndarray, half_len_m: float, half_wid_m: float,
              horizon_s: float = CRITICAL_HORIZON_S, step_s: float = CRITICAL_STEP_S):
    """Does each object's footprint overlap the ego's within the horizon, if both keep their
    current velocity?

    The box (+-half_len along, +-half_wid across) is laid in the ego's direction of travel, so a
    car in the next lane moving at the same speed is not a conflict but one closing along the
    same lane is. A (nearly) stationary ego uses a circle of radius half_len instead.

    Returns (hit (N,) bool, t_first_hit (N,) seconds or 0, min_dist (N,) metres over the horizon).
    """
    n = len(xy)
    if n == 0:
        return np.zeros(0, dtype=bool), np.zeros(0), np.zeros(0)
    ts = np.arange(0.0, horizon_s + 1e-9, step_s)
    rel = xy - np.asarray(ego_xy, dtype=float)
    rv = v - np.asarray(ego_v, dtype=float)
    pos = rel[:, None, :] + rv[:, None, :] * ts[None, :, None]         # (N, T, 2)
    dist = np.hypot(pos[..., 0], pos[..., 1])
    speed = float(np.hypot(ego_v[0], ego_v[1]))
    if speed > 0.5:
        ux, uy = ego_v[0] / speed, ego_v[1] / speed
        f = pos[..., 0] * ux + pos[..., 1] * uy
        lat = -pos[..., 0] * uy + pos[..., 1] * ux
        inside = (np.abs(f) < half_len_m) & (np.abs(lat) < half_wid_m)
    else:
        inside = dist < half_len_m
    hit = inside.any(axis=1)
    t_first = np.where(hit, ts[np.argmax(inside, axis=1)], 0.0)
    return hit, t_first, dist.min(axis=1)


class CautionMap:
    """One judge's caution layer: remembers markers between frames and scores them."""

    def __init__(self, margin_m: float = CAUTION_MARGIN_M,
                 lat_margin_m: float = CAUTION_LAT_MARGIN_M,
                 half_len_m: float = COLLISION_HALF_LEN_M,
                 half_wid_m: float = COLLISION_HALF_WID_M,
                 horizon_s: float = CRITICAL_HORIZON_S,
                 duplicate_radius_m: float = DUPLICATE_RADIUS_M,
                 ttl_s: float = CAUTION_TTL_S,
                 advisory_range_m: float = ADVISORY_RANGE_M,
                 score_min: float = CRITICAL_SCORE_MIN):
        self.margin_m = float(margin_m)
        self.lat_margin_m = float(lat_margin_m)
        self.half_len_m = float(half_len_m)
        self.half_wid_m = float(half_wid_m)
        self.advisory_range_m = float(advisory_range_m)
        self.score_min = float(score_min)
        self.horizon_s = float(horizon_s)
        self.duplicate_radius_m = float(duplicate_radius_m)
        self.ttl_s = float(ttl_s)
        # (sender, object id) -> (x, y, vx, vy, last_seen_t)
        self._markers: dict = {}

    @staticmethod
    def _without_ego(ego_xy, s: Sightings) -> Sightings:
        if not len(s):
            return s
        keep = np.hypot(s.xy[:, 0] - ego_xy[0], s.xy[:, 1] - ego_xy[1]) > EGO_SELF_RADIUS_M
        return Sightings(s.sender[keep], s.oid[keep], s.xy[keep], s.v[keep])

    def update(self, t: float, ego_xy, ego_v, confirmed: Sightings, unverified: Sightings,
               ego_seen_xy: Optional[np.ndarray] = None) -> FrameMap:
        """Fold this frame's confirmed / unverified sightings into the map.

        confirmed   objects admitted by the engine (trusted + corroborated)
        unverified  objects the engine flagged unverified
        ego_seen_xy positions the ego's own perception already covers (excluding itself)
        """
        confirmed = self._without_ego(ego_xy, confirmed)
        unverified = self._without_ego(ego_xy, unverified)
        covered = [a for a in (confirmed.xy, ego_seen_xy) if a is not None and len(a)]
        covered_xy = np.vstack(covered) if covered else np.empty((0, 2))

        for k in range(len(unverified)):
            key = (int(unverified.sender[k]), int(unverified.oid[k]))
            p = unverified.xy[k]
            if len(covered_xy) and float(np.min(np.hypot(covered_xy[:, 0] - p[0],
                                                         covered_xy[:, 1] - p[1]))) <= self.duplicate_radius_m:
                self._markers.pop(key, None)   # already covered by a trusted object
                continue
            self._markers[key] = (float(p[0]), float(p[1]),
                                  float(unverified.v[k, 0]), float(unverified.v[k, 1]), t)

        for key in [k for k, m in self._markers.items() if t - m[4] > self.ttl_s]:
            del self._markers[key]

        # Confirmed objects on a collision course (baseline hazard count, for comparison).
        n_crit_conf = 0
        if len(confirmed):
            hit, _, _ = conflicts(ego_xy, ego_v, confirmed.xy, confirmed.v,
                                  self.half_len_m, self.half_wid_m, self.horizon_s)
            n_crit_conf = int(np.count_nonzero(hit))

        markers = []
        keepout = self.half_len_m + self.margin_m
        if self._markers:
            keys = list(self._markers)
            arr = np.array([self._markers[k] for k in keys], dtype=float)
            age = t - arr[:, 4]
            xy = arr[:, 0:2] + arr[:, 2:4] * age[:, None]   # dead-reckon stale markers
            hit, tc, dmin = conflicts(ego_xy, ego_v, xy, arr[:, 2:4], keepout,
                                      self.half_wid_m + self.lat_margin_m, self.horizon_s)
            now_d = np.hypot(xy[:, 0] - ego_xy[0], xy[:, 1] - ego_xy[1])
            # Criticality: how close the marker is (1 at the ego, 0 at advisory_range_m) times how
            # soon the ego reaches it (1 now, 0 at the horizon). Off the ego's path it is 0, so a
            # car 90 m away on a crossing road, or one alongside that never conflicts, is not critical.
            proximity = np.clip(1.0 - now_d / self.advisory_range_m, 0.0, 1.0)
            urgency = np.where(hit, 1.0 - tc / self.horizon_s, 0.0)
            score = proximity * urgency
            for i, (sender, oid) in enumerate(keys):
                markers.append(CautionMarker(
                    sender=sender, object_id=oid, x=float(xy[i, 0]), y=float(xy[i, 1]),
                    vx=float(arr[i, 2]), vy=float(arr[i, 3]), keepout_m=keepout,
                    critical=bool(hit[i] and score[i] >= self.score_min), ttc_s=float(tc[i]),
                    min_dist_m=float(dmin[i]), dist_m=float(now_d[i]),
                    criticality=float(score[i])))
            markers.sort(key=lambda m: -m.criticality)

        if any(m.critical for m in markers):
            advisory = ADVISORY_CRITICAL
        elif any(m.dist_m <= self.advisory_range_m for m in markers):
            advisory = ADVISORY_CAUTION
        else:
            advisory = ADVISORY_CLEAR
        return FrameMap(tuple(markers), n_crit_conf, advisory)
