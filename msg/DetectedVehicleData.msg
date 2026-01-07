# Vehicle-specific data

sdsm_msgs/VehicleSize size
bool has_size

# Height (5 cm units, 0-127, 0=unavailable)
uint16 height
bool has_height

uint8 vehicle_class
bool has_vehicle_class

uint8 class_conf  # 0=unknown, 1-100=confidence %, 101=unavailable
bool has_class_conf
