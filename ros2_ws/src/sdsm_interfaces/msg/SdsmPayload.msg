# SdsmPayload.msg
#
# SAE J3224 / CPX-SDSM shaped payload with multi-object support.
# Modelled on veins_ros_v2v::messages::SdsmPayload for 802.11p transmission.
#
# Unit conventions:
#   offset_x/y/z, obj_width, obj_height  — 0.1 m per unit
#   obj_speed                             — 0.02 m/s per unit  (J2735)
#   obj_heading                           — 0.0125 deg per unit (J2735); 28800 = unavailable
#   ref_pos_x/y/z                         — metres directly (sim frame)

# === SDSM header ===
int32   msg_cnt             # 0-127, wraps at 128
int32[4] source_id          # 4-byte anonymized sender ID
int32   equipment_type      # 0=unknown 1=RSU 2=OBU 3=VRU

# === Reference position (sender), metres in sim coordinate frame ===
float64 ref_pos_x
float64 ref_pos_y
float64 ref_pos_z

# === Timestamp ===
int32  sdsm_year            # year CE
int32  sdsm_month           # 1-12
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
uint16[256] obj_measurement_time_ms  # CAPTURE LAG: ms between this object being
                              # measured and this message being sent, on the sender's
                              # own clock. A receiver recovers the measurement instant
                              # as (arrival - lag), which is how reports of one instant
                              # are grouped across senders whose processing chains
                              # finish at different times. 0 = sent at capture.
float32[256] obj_local_scores  # per-object local certainty in [0,1]; higher = more certain
                              # (factors in occlusion/range: high value + no witnesses may
                              #  simply mean the object is occluded from all other agents)
                              # OVERLAY on /perception/global_trustworthiness/trust_output:
                              # index 0 instead carries the sender's global_score (ego's R_new
                              # for that sender) and indices are zeroed — no per-object
                              # meaning on that topic, since there's no spare field to add one.

# === Frame-level trust score ===
float32 local_score   # frame-level certainty: mean of obj_local_scores for this
                       # message (0 if num_objects==0); set by the sender at publish time
