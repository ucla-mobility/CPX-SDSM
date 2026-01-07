# Detected object container

uint8 OPT_DATA_NONE = 0
uint8 OPT_DATA_VEHICLE = 1
uint8 OPT_DATA_VRU = 2
uint8 OPT_DATA_OBSTACLE = 3

sdsm_msgs/DetectedObjectCommonData det_obj_common

# CHOICE discriminator: only one of det_veh/det_vru/det_obst is valid
uint8 det_obj_opt_kind

sdsm_msgs/DetectedVehicleData det_veh
sdsm_msgs/DetectedVRUData det_vru
sdsm_msgs/DetectedObstacleData det_obst
