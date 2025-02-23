typedef enum {
    CAN_PACKET_SET_DUTY = 0, // Duty Cycle Mode
    CAN_PACKET_SET_CURRENT, // Current Loop Mode
    CAN_PACKET_SET_CURRENT_BRAKE, // Current Brake Mode
    CAN_PACKET_SET_RPM, // Speed Mode
    CAN_PACKET_SET_POS, // Position Mode
    CAN_PACKET_SET_ORIGIN_HERE, // Set origin position mode (zero mode)
    CAN_PACKET_SET_POS_SPD, // Position-Velocity Loop Mode
} CAN_AK60_PACKET_ID;

