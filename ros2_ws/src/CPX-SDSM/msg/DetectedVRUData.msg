# VRU (Vulnerable Road User) specific data

uint8 VRU_TYPE_UNAVAILABLE = 0
uint8 VRU_TYPE_PEDESTRIAN = 1
uint8 VRU_TYPE_PEDALCYCLIST = 2
uint8 VRU_TYPE_PUBLIC_SAFETY_WORKER = 3
uint8 VRU_TYPE_ANIMAL = 4

# VRU basic type (use VRU_TYPE_* constants)
# Classifies vulnerable road user: pedestrian, cyclist, worker, or animal
uint8 basic_type
