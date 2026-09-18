"""J2735/J3224 SDSM wire-format unit conversion factors.

Lives here (global_trust_tracker) rather than sdsm_interfaces to avoid
adding ament_cmake_python to the pure-interface package. Both
global_trust_tracker.tracker_node and global_trust_perception.perception_message
import from here; the dependency chain is:
    sdsm_interfaces <- global_trust_tracker <- global_trust_perception
so there is no cycle.
"""

OFFSET_UNIT_M    = 0.1       # offset_x/y/z, obj_width/length/height  -> metres
SPEED_UNIT_MS    = 0.02      # obj_speed                               -> m/s
HEADING_UNIT_DEG = 0.0125    # obj_heading                             -> degrees
HEADING_UNAVAILABLE = 28800  # J2735 sentinel: heading data not available
MEASUREMENT_TIME_UNIT_S = 0.001   # obj_measurement_time                -> seconds


def capture_lag_of(msg) -> float:
    """
    Decode this message's capture->send lag in seconds (>= 0).

    THE OTHER HALF OF THE LATENCY. perception_message.send_latency_of measures
    send->receive, the leg on the wire; this measures capture->send, the leg
    inside the sender (its perception chain). The two are deliberately separate
    quantities on separate clocks and must not be added into one number:

      - capture lag is stated by the sender on its own SCENE clock, so under
        use_sim_time it does not scale with playback rate. Frame GROUPING and
        the tracker's dt read it, because both must be reproducible.
      - send latency is measured across two hosts on the WALL clock, because
        a transmission delay is a physical fact that playback speed does not
        change. The FRESHNESS GATE reads it (kinematic_freshness), because
        that is what it bounds.

    Taken from detection 0: this sender publishes one synchronized sensor
    frame per message, so every populated entry holds the same value (see
    local_trust_estimation's sdsm_publisher). 0.0 for an empty message, and
    for any sender that leaves the field unset -- which reads as "sent at
    capture" and so preserves the pre-existing behaviour of grouping by
    arrival.

    Lives here rather than in either consumer because both packages decode it
    and this module is where shared wire-format knowledge belongs.
    """
    if int(msg.num_objects) <= 0:
        return 0.0
    return max(0.0, int(msg.obj_measurement_time_ms[0]) * MEASUREMENT_TIME_UNIT_S)
