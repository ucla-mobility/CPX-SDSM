# Detected object container

uint8 OPT_DATA_NONE = 0
uint8 OPT_DATA_VEHICLE = 1
uint8 OPT_DATA_VRU = 2
uint8 OPT_DATA_OBSTACLE = 3

sdsm_msgs/DetectedObjectCommonData det_obj_common

# CHOICE discriminator: only one of det_veh/det_vru/det_obst is valid
# Check this field before reading type-specific data
# 0=none, 1=vehicle, 2=VRU, 3=obstacle
uint8 det_obj_opt_kind

# Vehicle-specific data (valid only when det_obj_opt_kind == OPT_DATA_VEHICLE)
sdsm_msgs/DetectedVehicleData det_veh

# VRU-specific data (valid only when det_obj_opt_kind == OPT_DATA_VRU)
sdsm_msgs/DetectedVRUData det_vru

# Obstacle-specific data (valid only when det_obj_opt_kind == OPT_DATA_OBSTACLE)
sdsm_msgs/DetectedObstacleData det_obst
