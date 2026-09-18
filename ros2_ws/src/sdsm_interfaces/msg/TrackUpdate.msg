# TrackUpdate.msg
# SORT tracker output: per-detection track state, parallel to the source SDSM.
# Arrays have length equal to SdsmPayload.num_objects for the same message.

# Correlation with the originating SdsmPayload
int32[4] source_id          # matches SdsmPayload.source_id
int32    msg_cnt             # matches SdsmPayload.msg_cnt

# Per-detection track state (arrays of length num_objects)
int32[]  track_id            # persistent object ID across frames
int32[]  age                 # frames since track was initialized
bool[]   confirmed           # True after min_hits consecutive hits
int32[]  time_since_update   # frames since last matched detection

# Kalman state estimates (metres, m/s)
float32[] kalman_x
float32[] kalman_y
float32[] kalman_vx
float32[] kalman_vy
float32[] kalman_w           # width
float32[] kalman_l           # length
float32[] kalman_h           # height (EMA)
