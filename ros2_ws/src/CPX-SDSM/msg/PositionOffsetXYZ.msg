# Position offset relative to refPos
# Relative coordinates avoid coordinate system conversions and improve precision

# X offset (0.1 m units)
# Lateral offset from sender (left/right)
int16 offset_x

# Y offset (0.1 m units)
# Longitudinal offset from sender (forward/backward)
int16 offset_y

# Z offset (0.1 m units)
# Vertical offset; optional because most objects are on ground plane
int16 offset_z
bool has_offset_z
