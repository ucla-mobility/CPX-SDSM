# SAE J3224 Sensor Data Sharing Message

uint8 EQUIPMENT_TYPE_UNKNOWN = 0
uint8 EQUIPMENT_TYPE_RSU = 1
uint8 EQUIPMENT_TYPE_OBU = 2
uint8 EQUIPMENT_TYPE_VRU = 3

# Message count (0-127)
uint8 msg_cnt

# Temporary ID of sender (4 bytes)
uint8[4] source_id

uint8 equipment_type

sdsm_msgs/DDateTime sdsm_time_stamp
sdsm_msgs/Position3D ref_pos

# Detected objects (1-256 per J3224)
sdsm_msgs/DetectedObjectData[] objects
