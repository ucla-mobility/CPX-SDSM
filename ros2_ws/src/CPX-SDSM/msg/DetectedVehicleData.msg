# Vehicle-specific data

# Vehicle dimensions (width/length in cm)
# Optional per ASN.1; check has_size before use
sdsm_msgs/VehicleSize size
bool has_size

# Vehicle height (5 cm units, 0-127, 0=unavailable)
# Separate from size because height uses different units per J2735
uint16 height
bool has_height

# Basic vehicle class (per J2735 BasicVehicleClass)
# Categories: passenger, truck, bus, motorcycle, etc.
uint8 vehicle_class
bool has_vehicle_class

uint8 class_conf  # 0=unknown, 1-100=confidence %, 101=unavailable
bool has_class_conf
