# mypy: ignore-errors
# sdsm_interfaces ships generated message classes; mypy cannot resolve them.
# A ROS-message boundary, excluded from type checking on purpose — same
# rationale as perception_message.py (see test/test_mypy.py).
"""
Fused-scene codec: the ONE place that knows the fused-box wire format.

Information-hiding boundary around sdsm_interfaces/FusedObjects, mirroring
what trust_verdicts.py does for the verdict channel and perception_message.py
does for the perception messages.

WHAT THIS CHANNEL IS. Vehicle-internal diagnostics: the consensus boxes MS-PSF
produced this frame, so a visualization can draw the fused estimate next to
the inputs that produced it. It is NOT the trust pipeline's output — a fused
box has no sender and no reputation, and nothing in the production path may
consume it.

COSTS NOTHING WHEN UNWATCHED. fuse() already builds this geometry on every
frame and discards it (see mmcooper_fuse/adapter.py's display_derivation
note), so there is no extra computation here — only packing, which the
publisher skips when no one is subscribed.

    Message               - the message type (pub/sub + type hints)
    topic(ego_id)         - the per-ego topic it travels on
    ego_id_from_topic()   - inverse of topic()
    build(...)            - encode one FusionResult
    boxes_of(msg)         - decode to (center, size, yaw, score, label) tuples
"""

import math

from sdsm_interfaces.msg import FusedObjects


# --- Public, format-agnostic handles -------------------------------------
Message = FusedObjects
TOPIC_PREFIX = '/perception/global_trustworthiness/viz/fused_boxes'

# fuse() states each box as four BEV corners in this order (see
# mmcooper_fuse.geometry.corners_from_pose):
#     0:(+L/2, +W/2)  1:(+L/2, -W/2)  2:(-L/2, -W/2)  3:(-L/2, +W/2)
# so corner0 - corner3 spans the LENGTH axis and gives the heading, while
# corner0 - corner1 spans the WIDTH. Decoding here rather than at the call
# site keeps the corner convention in one place.
_LENGTH_FROM = (0, 3)
_WIDTH_FROM = (0, 1)


def topic(ego_id: int) -> str:
    """
    Per-ego fused-scene topic: the shared prefix with the fusing ego's id.

    Fusion is per-ego — it combines that ego's own detections with the peers
    it heard — so two egos produce different boxes for the same scene. The
    identity rides in the topic NAME, matching trust_verdicts.topic().
    """
    return f'{TOPIC_PREFIX}/agent_{ego_id}'


def ego_id_from_topic(topic_name: str) -> int | None:
    """Inverse of topic(); None if topic_name isn't a fused-scene topic."""
    prefix = f'{TOPIC_PREFIX}/agent_'
    if not topic_name.startswith(prefix):
        return None
    try:
        return int(topic_name[len(prefix):])
    except ValueError:
        return None


def _pose_of(corners) -> tuple[float, float, float, float]:
    """(center_x, center_y, length, yaw) from four BEV corners."""
    cx = sum(float(c[0]) for c in corners) / len(corners)
    cy = sum(float(c[1]) for c in corners) / len(corners)
    a, b = _LENGTH_FROM
    dx = float(corners[a][0]) - float(corners[b][0])
    dy = float(corners[a][1]) - float(corners[b][1])
    return cx, cy, math.hypot(dx, dy), math.atan2(dy, dx)


def _width_of(corners) -> float:
    """Box width (metres) from four BEV corners."""
    a, b = _WIDTH_FROM
    return math.hypot(float(corners[a][0]) - float(corners[b][0]),
                      float(corners[a][1]) - float(corners[b][1]))


def build(fusion, ego_source_id) -> Message:
    """
    Encode one frame's FusionResult.

    `fusion` is trustworthy_perception.TrustEngine.last_fusion — read-only, and
    valid only for the frame just processed. An empty or absent fusion encodes
    as num_objects=0 rather than raising, so a quiet frame publishes an empty
    scene and a viewer clears rather than holding the last one forever.
    """
    msg = Message()
    msg.ego_source_id = list(ego_source_id)
    if fusion is None or not fusion.fused_boxes:
        msg.num_objects = 0
        return msg

    count = len(fusion.fused_boxes)
    msg.num_objects = count
    for i in range(count):
        cx, cy, length, yaw = _pose_of(fusion.fused_boxes[i])
        msg.center_x.append(cx)
        msg.center_y.append(cy)
        # BEV fusion carries z and height rather than fusing them; the adapter
        # derives both alongside the box (fused_z_and_height).
        msg.center_z.append(float(fusion.fused_centers_z[i]))
        msg.size_x.append(length)
        msg.size_y.append(_width_of(fusion.fused_boxes[i]))
        msg.size_z.append(float(fusion.fused_heights[i]))
        msg.yaw.append(yaw)
        msg.score.append(float(fusion.fused_scores[i]))
        msg.label.append(int(fusion.fused_labels[i]))
    return msg


def boxes_of(msg: Message):
    """Decode to [((x, y, z), (size_x, size_y, size_z), yaw, score, label)]."""
    return [
        ((msg.center_x[i], msg.center_y[i], msg.center_z[i]),
         (msg.size_x[i], msg.size_y[i], msg.size_z[i]),
         msg.yaw[i], msg.score[i], int(msg.label[i]))
        for i in range(int(msg.num_objects))
    ]
