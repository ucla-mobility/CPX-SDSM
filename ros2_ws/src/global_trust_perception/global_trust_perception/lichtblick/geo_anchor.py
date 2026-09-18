"""
The geo->world projection for the Lichtblick overlays: lat/lon -> world metres.

Nothing in the recordings is georeferenced -- the SDSM payload carries ref_pos
in metres in an arbitrary sim frame (SdsmPayload.msg), and the bags carry no
GNSS at all, so `world` is the recording's own localization root with no tie to
Earth (the same fact ground_backdrop_node's docstring states about the aerial).
A map drawn in `world` therefore has to be PLACED by hand.

lonlat_to_world(...) is the single place that geo->world mapping lives, and it is
fully PARAMETERIZED and pure -- no ROS, no state, no baked constants. The caller
supplies both the anchor (the lat/lon that maps to center_x/center_y) and the
alignment. lanelet_overlay_node anchors on the clipped map's own CENTROID, so the
map is centred at center_x/center_y by construction; center_x/center_y then drag
it, yaw_deg rotates it in place, and scale sizes it. There is no hand-guessed
reference lat/lon left to be wrong -- an earlier version pinned a fixed guessed
anchor and a ~40 m error in it threw the whole overlay off and swung it around a
bad pivot when yaw was tuned.

CONVENTIONS
- Equirectangular about the anchor: exact enough over the ~100 m a clipped
  intersection spans, and it needs no projection library (this must import on the
  host too, where the clip tool runs).
- yaw_deg matches ground_backdrop's convention (about world +Z, image/map north
  onto the frame), so the aerial and this overlay take the SAME yaw when both
  sit in `world`. scale is a metric correction, 1.0 when world is true-metre.
"""

import math

# Local metres per degree latitude -- near-constant, so it stays fixed. The
# longitude factor shrinks by cos(latitude) and is computed per-call from the
# anchor, since the anchor is now a parameter rather than a constant.
_M_PER_DEG_LAT = 110540.0


def lonlat_to_world(lat, lon, anchor_lat, anchor_lon, center_x, center_y,
                    yaw_deg, scale=1.0):
    """Project one geographic point into the `world` frame, returning (x, y).

    anchor_lat/anchor_lon is the reference point that maps to (center_x,
    center_y) -- the overlay node passes the clipped map's centroid, so the map
    centres on (center_x, center_y). yaw_deg rotates about that centre (world
    +Z); scale is a residual metric correction (1.0 when `world` is true-metre).

    Steps: geographic offset -> local ENU metres about the anchor; rotate ENU
    into the tilted frame; scale; translate to the centre.
    """
    m_per_deg_lon = 111320.0 * math.cos(math.radians(anchor_lat))
    east_m = (lon - anchor_lon) * m_per_deg_lon
    north_m = (lat - anchor_lat) * _M_PER_DEG_LAT
    yaw = math.radians(yaw_deg)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    x = center_x + scale * (east_m * cos_y - north_m * sin_y)
    y = center_y + scale * (east_m * sin_y + north_m * cos_y)
    return (x, y)
