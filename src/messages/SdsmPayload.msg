namespace veins_ros_v2v::messages;

// SAE J3224 / CPX-SDSM shaped payload with multi-object support.
// Carried in DemoSafetyMessage for 802.11p transmission.
// Legacy fields (senderNodeIndex, senderX, ...) kept for greedy/AoI tracking.

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

    // Reference position (sender) — meters in sim
    double ref_pos_x = 0;
    double ref_pos_y = 0;
    double ref_pos_z = 0;

    // Timestamp (CPX DDateTime)
    int sdsm_day = 1;              // 1-31
    long sdsm_time_of_day_ms = 0;   // ms since midnight

    // === Detected objects array (up to 16 per J3224) ===
    int numObjects = 0;            // actual count of objects in this message (0..16)
    int obj_type[16];              // 1=vehicle per object
    int object_id[16];             // node index of detected object
    int offset_x[16];              // position offset from ref_pos, 0.1 m units
    int offset_y[16];
    int offset_z[16];
    int obj_speed[16];             // 0.02 m/s units (J2735)
    int obj_heading[16];           // 0.0125 deg units, 28800=unavailable
}
