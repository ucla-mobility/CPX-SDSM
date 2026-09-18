# SdsmTrustOutput.msg
#
# Trust-pipeline OUTPUT: one sender's SDSM, rebroadcast by an ego that has
# judged it, on /perception/global_trustworthiness/trust_output/agent_<ego_id>.
# The rebroadcasting ego's identity is carried by the topic NAME, not by this
# payload, so a consumer learns whose trust view a rebroadcast represents from
# the channel it arrived on.
#
# Relationship to SdsmPayload (the INPUT message every agent broadcasts):
#   - the object payload below is identical, field for field
#   - obj_local_scores[32] is DROPPED. Per-object local certainty is the
#     sender's own view of its detections; it has no meaning once the objects
#     have been judged, and it costs 128 bytes on an 802.11p link
#   - global_score is ADDED: the rebroadcasting ego's reputation for the sender
#
# INVARIANT: this message is only ever published for a sender that PASSED the
# rebroadcasting ego's trust gate. An untrusted sender's data is never emitted
# here — the trusted set is expressed by PRESENCE on the topic, not by a flag,
# so a downstream consumer cannot accidentally act on withheld perception.
# Visualization that needs to show untrusted senders uses TrustVerdicts.msg,
# which is vehicle-internal and never transmitted.
#
# Unit conventions are identical to SdsmPayload.msg:
#   offset_x/y/z, obj_width, obj_height  — 0.1 m per unit
#   obj_speed                             — 0.02 m/s per unit  (J2735)
#   obj_heading                           — 0.0125 deg per unit (J2735); 28800 = unavailable
#   ref_pos_x/y/z                         — metres directly (sim frame)

# === SDSM header (the ORIGINAL sender's, carried through) ===
int32   msg_cnt             # 0-127, wraps at 128
int32[4] source_id          # 4-byte anonymized sender ID
int32   equipment_type      # 0=unknown 1=RSU 2=OBU 3=VRU

# === Reference position (sender), metres in sim coordinate frame ===
float64 ref_pos_x
float64 ref_pos_y
float64 ref_pos_z

# === Timestamp ===
int32  sdsm_day             # 1-31
int64  sdsm_time_of_day_ms  # ms since midnight

# === Detected objects (flat parallel arrays; valid indices 0 .. num_objects-1) ===
int32 num_objects           # actual count of objects in this message (0..32)

int32[256]  obj_type         # 1=vehicle, per J3224 ObjectType
int32[256]  object_id        # node index / ID of the detected object
int32[256]  offset_x         # position offset from ref_pos, 0.1 m units
int32[256]  offset_y
int32[256]  offset_z
int32[256]  obj_width        # 0.1 m units
int32[256]  obj_length       # 0.1 m units
int32[256]  obj_height       # 0.1 m units
int32[256]  obj_speed        # 0.02 m/s units (J2735)
int32[256]  obj_heading      # 0.0125 deg units; 28800 = unavailable
uint16[256] obj_measurement_time_ms  # CAPTURE LAG, ms before this message's send
                              # stamp; see SdsmPayload.msg. Passed through unchanged.

# === Trust scores (both singular; replace SdsmPayload's obj_local_scores[32]) ===
float32 local_score    # the SENDER's own frame-level certainty, passed through
                       # unchanged from the input message
float32 global_score   # the rebroadcasting EGO's reputation R for this sender.
                       # Kept separate from local_score rather than sent as one
                       # pre-multiplied product: a product cannot be decomposed
                       # again, so a consumer that wants to weigh the two
                       # differently could not recover them.
