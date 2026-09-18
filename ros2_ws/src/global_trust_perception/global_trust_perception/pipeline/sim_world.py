"""
Ground-truth world + visibility model for the intersection sim.

Single source of truth for WHERE everything is and WHO can see WHAT. The
perception-message codec (`perception_message.build`) consumes plain `Detection`
tuples from `observe()` and never reaches into this module's geometry, so the
wire format and the simulated scene evolve independently (SRP + information
hiding).

Scene: an unmarked intersection. Four cars (sensors that also get seen) drive in
straight lines, one RSU watches from the corner (a sensor, never a target), and
one pedestrian (VRU) stands still (a target, never a sensor). Each sensor
independently reports the subset of OTHER targets it can currently see; when two
sensors report the same target, that agreement is what the trust pipeline scores.

Frame of reference: metres, J2735 heading (0 = +y / North, clockwise). One tick
advances a moving entity by `STEP_PER_TICK_M` along its heading.

Visibility of a target from a sensor requires ALL of:
  * within `DETECT_RANGE_M`,
  * inside the sensor's forward OR backward FOV cone (`FOV_HALF_ANGLE`); an
    omnidirectional sensor (the RSU) skips this,
  * not occluded: the VRU is hidden from a car until that car has reached the
    corner mark `VRU_REVEAL_MARK` (the RSU always sees it).

Cars start spaced out along their own lane (same heading, same convergence
point), so `observe()` derives every tick — including tick 0 — purely from the
rules above: most pairs start out of range/FOV and reveal each other as they
approach, rather than everything already being visible from the start. Car 1
additionally reports a persistent phantom nobody else ever sees.
"""

import math
from typing import NamedTuple, Optional

# --- object / equipment type codes (J3224) -------------------------------
OBJ_TYPE_VEHICLE = 1
OBJ_TYPE_PERSON  = 2
EQUIP_RSU = 1
EQUIP_OBU = 2

# --- timing & motion -----------------------------------------------------
# Real-life cadence: 2 Hz (0.5 s/frame), matching the production FRAME_WINDOW_MS
# and agent._TICK_INTERVAL_S. Spaced-out cars take up to 81 ticks to reveal
# everything (Car 3 first sees the VRU), so the full scenario runs ~40 s.
TICK_DT_S       = 0.5            # seconds per tick; matches agent._TICK_INTERVAL_S
STEP_PER_TICK_M = 0.2            # distance a moving entity advances each tick

# --- visibility tuning ---------------------------------------------------
DETECT_RANGE_M    = 4.5          # a target beyond this is invisible to any sensor
FOV_HALF_ANGLE    = 60.0         # forward AND backward cone half-width (deg)
VRU_REVEAL_MARK   = (1.0, -1.0)  # corner a car must reach to clear the occluder
VRU_REVEAL_RADIUS = 0.6          # how close to the mark counts as "reached"

# --- per-object local certainty (0..1); real boxes wobble in [0.8, 0.9] --
_SCORE_BASE  = 0.85
_SCORE_NOISE = 0.05
_GHOST_SCORE = 0.90   # Car 1's phantom: confident on purpose, yet uncorroborated

# Standard car footprint (width, length, height); VRU pedestrian footprint.
_CAR_WLH = (2.0, 4.5, 1.5)
_VRU_WLH = (0.6, 0.6, 1.7)


class _Entity(NamedTuple):
    """One thing in the world; a sensor, a target, or both."""

    start: tuple[float, float]      # position at tick 0, metres
    heading_deg: Optional[float]    # travel heading; None = stationary
    is_sensor: bool                 # broadcasts an SDSM (runs an agent node)
    is_target: bool                 # can be detected by sensors
    obj_type: int                   # J3224 object type when reported as a target
    size_wlh: tuple[float, float, float]  # width, length, height (m)
    equipment: int                  # equipment_type it broadcasts as (sensor)
    omni: bool = False              # 360-degree sensor (infrastructure)


_VRU_ID = 6

# Cars start pushed back along their own lane (same heading, same convergence
# point) so pairwise range/FOV reveals are visible as the scene plays out,
# instead of everything already being in range at tick 0.
_WORLD: dict[int, _Entity] = {
    1: _Entity((0.0,  6.0), 180.0, True,  True,  OBJ_TYPE_VEHICLE, _CAR_WLH, EQUIP_OBU),
    2: _Entity((1.0, -6.0),   0.0, True,  True,  OBJ_TYPE_VEHICLE, _CAR_WLH, EQUIP_OBU),
    3: _Entity((1.0, -9.0),   0.0, True,  True,  OBJ_TYPE_VEHICLE, _CAR_WLH, EQUIP_OBU),
    4: _Entity((-6.0, -1.0), 90.0, True,  True,  OBJ_TYPE_VEHICLE, _CAR_WLH, EQUIP_OBU),
    5: _Entity((0.0,  1.0),  None, True,  False, OBJ_TYPE_VEHICLE, _CAR_WLH, EQUIP_RSU,
               omni=True),
    _VRU_ID: _Entity((2.0, -1.5), None, False, True, OBJ_TYPE_PERSON, _VRU_WLH,
                     EQUIP_OBU),
}

# Car 1's persistent phantom: a fixed ghost at the reveal mark that no other
# sensor ever corroborates, so it drives the deferred-ledger path.
_PHANTOM_SENSOR = 1
_PHANTOM_POS    = (1.0, -1.0)
_PHANTOM_ID     = 99


class Detection(NamedTuple):
    """One reported object, in metres/degrees; `perception_message` packs it."""

    object_id: int
    x: float
    y: float
    width: float
    length: float
    height: float
    obj_type: int
    speed_mps: float
    heading_deg: Optional[float]   # None = heading unavailable (stationary)
    local_score: float


# --- geometry ------------------------------------------------------------

def _dir_unit(heading_deg: float) -> tuple[float, float]:
    """Unit velocity direction for a J2735 heading (0 = +y, clockwise)."""
    r = math.radians(heading_deg)
    return (math.sin(r), math.cos(r))


# --- TEMP: quick stop-at-intersection pause (delete this block + the two TEMP
# lines in pos_at below to go back to constant motion) -------------------------
# Only the N/S cars (1-3) yield; Car 4 crosses on the other phase, so holding
# 1-3 is what keeps it from colliding with the cross-traffic car.
_STOP_IDS   = {1, 2, 3}
_STOP_TICK  = 10   # cars 1-3 freeze 5 s in (10 frames @ 2 Hz)
_STOP_DWELL = 50   # frames held at a standstill before motion resumes


def _travel_ticks(tick: int) -> int:
    """Effective motion ticks with one pause at the intersection (TEMP)."""
    if tick <= _STOP_TICK:
        return tick
    if tick <= _STOP_TICK + _STOP_DWELL:
        return _STOP_TICK
    return tick - _STOP_DWELL
# --- END TEMP -----------------------------------------------------------------


def pos_at(entity_id: int, tick: int) -> tuple[float, float]:
    """Position of an entity at a given tick (stationary entities ignore tick)."""
    e = _WORLD[entity_id]
    if e.heading_deg is None:
        return e.start
    dx, dy = _dir_unit(e.heading_deg)
    d = tick * STEP_PER_TICK_M
    if entity_id in _STOP_IDS:                  # TEMP: hold 1-3 while 4 crosses
        d = _travel_ticks(tick) * STEP_PER_TICK_M
    return (e.start[0] + dx * d, e.start[1] + dy * d)


def _speed_mps(entity_id: int) -> float:
    """Reported speed: STEP_PER_TICK_M per tick, or 0 for a stationary entity."""
    return 0.0 if _WORLD[entity_id].heading_deg is None else STEP_PER_TICK_M / TICK_DT_S


def _bearing_deg(from_xy: tuple[float, float], to_xy: tuple[float, float]) -> float:
    """Compass bearing from one point to another, J2735 convention (0 = +y, CW)."""
    dx = to_xy[0] - from_xy[0]
    dy = to_xy[1] - from_xy[1]
    return math.degrees(math.atan2(dx, dy)) % 360.0


def _ang_diff(a: float, b: float) -> float:
    """Smallest absolute difference between two headings, in [0, 180]."""
    d = abs((a - b) % 360.0)
    return min(d, 360.0 - d)


def _in_fov(sensor_id: int, sensor_xy: tuple[float, float],
            target_xy: tuple[float, float]) -> bool:
    """Whether a target lies in the sensor's forward or backward FOV cone."""
    e = _WORLD[sensor_id]
    if e.omni or e.heading_deg is None:
        return True
    bearing = _bearing_deg(sensor_xy, target_xy)
    forward  = _ang_diff(bearing, e.heading_deg) <= FOV_HALF_ANGLE
    backward = _ang_diff(bearing, (e.heading_deg + 180.0) % 360.0) <= FOV_HALF_ANGLE
    return forward or backward


def _reached_mark(sensor_id: int, tick: int) -> bool:
    """
    Whether a moving sensor has ever come within VRU_REVEAL_RADIUS of the mark.

    Latched: because a car travels in a straight line, once the closest-approach
    tick has passed the reveal is decided by the (constant) perpendicular distance
    to the mark; before it, by the current distance. So a car that clears the
    corner keeps its line of sight for the rest of the run.
    """
    e = _WORLD[sensor_id]
    if e.heading_deg is None:
        return False
    dirx, diry = _dir_unit(e.heading_deg)
    mx = VRU_REVEAL_MARK[0] - e.start[0]
    my = VRU_REVEAL_MARK[1] - e.start[1]
    along_m = mx * dirx + my * diry            # metres from start to closest approach
    if along_m <= 0.0:                         # mark is behind the start of travel
        return math.dist(e.start, VRU_REVEAL_MARK) <= VRU_REVEAL_RADIUS
    if tick * STEP_PER_TICK_M >= along_m:      # closest approach already passed
        perp = abs(mx * diry - my * dirx)      # |cross| = perpendicular distance
        return perp <= VRU_REVEAL_RADIUS
    return math.dist(pos_at(sensor_id, tick), VRU_REVEAL_MARK) <= VRU_REVEAL_RADIUS


def _visible(sensor_id: int, target_id: int, tick: int) -> bool:
    """Whether sensor_id can see target_id this tick (range + FOV + occlusion)."""
    if target_id == sensor_id or not _WORLD[target_id].is_target:
        return False
    s_xy = pos_at(sensor_id, tick)
    t_xy = pos_at(target_id, tick)
    if math.dist(s_xy, t_xy) > DETECT_RANGE_M:
        return False
    if not _in_fov(sensor_id, s_xy, t_xy):
        return False
    if target_id == _VRU_ID and _WORLD[sensor_id].heading_deg is not None:
        # a car only sees the VRU once it has cleared the corner; the RSU always does
        return _reached_mark(sensor_id, tick)
    return True


# --- reporting -----------------------------------------------------------

def _local_score(seed: int, tick: int) -> float:
    """Per-object certainty wobbling deterministically within [0.8, 0.9]."""
    return _SCORE_BASE + _SCORE_NOISE * math.sin(tick * 1.3 + seed)


def _detection_for(target_id: int, tick: int) -> Detection:
    """Build a Detection for a real world target at this tick."""
    e = _WORLD[target_id]
    x, y = pos_at(target_id, tick)
    w, l, h = e.size_wlh
    return Detection(
        object_id=target_id, x=x, y=y,
        width=w, length=l, height=h,
        obj_type=e.obj_type,
        speed_mps=_speed_mps(target_id),
        heading_deg=e.heading_deg,
        local_score=_local_score(target_id, tick),
    )


def _phantom(tick: int) -> Detection:
    """Car 1's persistent, never-corroborated ghost at the reveal mark."""
    w, l, h = _CAR_WLH
    return Detection(
        object_id=_PHANTOM_ID, x=_PHANTOM_POS[0], y=_PHANTOM_POS[1],
        width=w, length=l, height=h,
        obj_type=OBJ_TYPE_VEHICLE, speed_mps=0.0, heading_deg=None,
        local_score=_GHOST_SCORE,
    )


def observe(sensor_id: int, tick: int) -> tuple[tuple[float, float], list[Detection]]:
    """
    Return (sensor_xy, detections) for one sensor at one tick.

    Every tick, including tick 0, uses the same range + FOV + occlusion rules
    (cars start far enough apart that tick 0 has few or no sightings, by design —
    see the reveal-timing comment on _WORLD). Car 1's persistent phantom is
    appended every tick. A non-sensor (or unknown) id yields no detections.
    """
    e = _WORLD.get(sensor_id)
    sensor_xy = pos_at(sensor_id, tick) if e is not None else (0.0, 0.0)
    if e is None or not e.is_sensor:
        return sensor_xy, []

    target_ids = [t for t in _WORLD if _visible(sensor_id, t, tick)]
    detections = [_detection_for(t, tick) for t in target_ids]
    if sensor_id == _PHANTOM_SENSOR:
        detections.append(_phantom(tick))
    return sensor_xy, detections


def equipment_type_of(sensor_id: int) -> int:
    """Equipment type a sensor broadcasts as (RSU vs OBU); OBU for unknown ids."""
    e = _WORLD.get(sensor_id)
    return e.equipment if e is not None else EQUIP_OBU
