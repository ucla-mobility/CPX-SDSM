namespace veins_ros_v2v::messages;

// SAE J3224 / CPX-SDSM shaped payload (mirrors ros2_ws/src/CPX-SDSM SensorDataSharingMessage + one DetectedObjectData).
// Carried in DemoSafetyMessage (BSM). Same structure as real-world SDSM for ROS 2 / bridge compatibility.
// Legacy fields (senderNodeIndex, senderX, ...) kept for greedy/AoI and backward compatibility.

packet SdsmPayload
{
    string payload;

    // === Legacy fields (greedy, AoI, neighbor tracking) ===
    int    senderNodeIndex = -1;
    double senderX         = 0;
    double senderY         = 0;
    double senderSpeed     = 0;
    double senderHeading   = 0;
    double sendTimestamp   = 0;
    long   message_id      = 0;   // global id to link TX and RX events

    // === SDSM header (CPX-SDSM SensorDataSharingMessage) ===
    int msg_cnt = 0;              // 0-127, wraps
    int source_id[4];             // 4-byte anonymized sender ID
    int equipment_type = 2;        // 0=unknown, 1=RSU, 2=OBU, 3=VRU

    // Reference position (sender) — meters in sim (CPX Position3D uses lat/lon; we use x,y,z for SUMO)
    double ref_pos_x = 0;
    double ref_pos_y = 0;
    double ref_pos_z = 0;

    // Timestamp (CPX DDateTime)
    int sdsm_day = 1;              // 1-31
    long sdsm_time_of_day_ms = 0;   // ms since midnight

    // === One detected object (CPX DetectedObjectData: common + vehicle) ===
    int obj_type = 1;              // 1=vehicle
    int obj_type_cfd = 100;
    int object_id = 0;
    int measurement_time = 0;      // ms offset from sdsm_time
    int offset_x = 0;              // 0.1 m units
    int offset_y = 0;
    int offset_z = 0;
    int speed = 0;                 // 0.02 m/s units (J2735)
    int heading = 0;               // 0.0125 deg units, 28800=unavailable
    int det_obj_opt_kind = 1;      // 1=vehicle
    bool has_vehicle_size = true;
    int vehicle_width_cm = 180;
    int vehicle_length_cm = 450;
}
