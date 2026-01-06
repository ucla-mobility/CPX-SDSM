# Common data for detected object

uint8 OBJ_TYPE_UNKNOWN = 0
uint8 OBJ_TYPE_VEHICLE = 1
uint8 OBJ_TYPE_VRU = 2
uint8 OBJ_TYPE_ANIMAL = 3
uint8 OBJ_TYPE_OBSTACLE = 4

uint8 obj_type
uint8 obj_type_cfd  # 0=unknown, 1-100=confidence %, 101=unavailable
uint16 object_id

# Measurement time offset from sDSMTimeStamp (ms, -1500..1500)
int16 measurement_time

sdsm_msgs/PositionOffsetXYZ pos
sdsm_msgs/PositionConfidenceSet pos_confidence

# Ground speed (0.02 m/s units, 8191=unavailable)
uint16 speed

# Z-axis speed (0.02 m/s units, 8191=unavailable)
uint16 speed_z
bool has_speed_z

# Heading (0.0125 deg units, 0-28799, 28800=unavailable)
uint16 heading
