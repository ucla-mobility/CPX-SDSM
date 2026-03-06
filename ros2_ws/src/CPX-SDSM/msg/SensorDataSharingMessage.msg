# SAE J3224 Sensor Data Sharing Message

uint8 EQUIPMENT_TYPE_UNKNOWN = 0
uint8 EQUIPMENT_TYPE_RSU = 1
uint8 EQUIPMENT_TYPE_OBU = 2
uint8 EQUIPMENT_TYPE_VRU = 3

# Message count (0-127, wraps per J2735)
# Sequence number for message ordering and duplicate detection
uint8 msg_cnt

# Temporary ID of sender (4 bytes)
# Anonymized identifier for privacy; changes periodically
uint8[4] source_id

# Equipment type (use EQUIPMENT_TYPE_* constants)
# Identifies sender as RSU, OBU, or VRU device
uint8 equipment_type

sdsm_msgs/DDateTime sdsm_time_stamp
sdsm_msgs/Position3D ref_pos

# Detected objects (1-256 per J3224)
# Array length must be validated: 1 <= len(objects) <= 256
sdsm_msgs/DetectedObjectData[] objects
