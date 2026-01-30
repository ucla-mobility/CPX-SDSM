# Common data for detected object

uint8 OBJ_TYPE_UNKNOWN = 0
uint8 OBJ_TYPE_VEHICLE = 1
uint8 OBJ_TYPE_VRU = 2
uint8 OBJ_TYPE_ANIMAL = 3
uint8 OBJ_TYPE_OBSTACLE = 4

# Object type (use OBJ_TYPE_* constants)
# High-level classification: vehicle, VRU, animal, or obstacle
uint8 obj_type

# Classification confidence (0=unknown, 1-100=confidence %, 101=unavailable)
uint8 obj_type_cfd

# Temporary object ID assigned by source
# Unique within this message; used for tracking objects across multiple SDSM messages
uint16 object_id

# Measurement time offset from sDSMTimeStamp (ms, -1500..1500)
# Relative to message timestamp; allows objects detected at different times
# Negative = detected before message timestamp, positive = after
int16 measurement_time

# Position offset relative to sender's refPos
# Uses relative coordinates for efficiency and coordinate system independence
sdsm_msgs/PositionOffsetXYZ pos

# Position confidence values (per J2735)
# Indicates accuracy of position measurements
sdsm_msgs/PositionConfidenceSet pos_confidence

# Ground speed (0.02 m/s units, 8191=unavailable)
uint16 speed

# Z-axis speed (0.02 m/s units, 8191=unavailable)
# Vertical velocity; optional because most objects move on ground plane
uint16 speed_z
bool has_speed_z

# Heading (0.0125 deg units, 0-28799, 28800=unavailable)
# Direction of travel in degrees (0-359.9875°); 0.0125° precision per J2735
uint16 heading
