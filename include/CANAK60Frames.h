#include <stdint.h>

#define CAN_AK60_MAKE_ARBITRATION(PACKET_ID, MOTOR_ID) ((uint8_t)MOTOR_ID | ((uint32_t)PACKET_ID) << 8)

typedef enum {
    CAN_AK60_PACKET_SET_DUTY = 0, // Duty Cycle Mode
    CAN_AK60_PACKET_SET_CURRENT, // Current Loop Mode
    CAN_AK60_PACKET_SET_CURRENT_BRAKE, // Current Brake Mode
    CAN_AK60_PACKET_SET_RPM, // Speed Mode
    CAN_AK60_PACKET_SET_POS, // Position Mode
    CAN_AK60_PACKET_SET_ORIGIN_HERE, // Set origin position mode (zero mode)
    CAN_AK60_PACKET_SET_POS_SPD, // Position-Velocity Loop Mode
} CAN_AK60_PACKET_ID;

typedef uint32_t ak60_position_setpoint_t;