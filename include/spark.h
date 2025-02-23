#include <stdint.h>

#define SPARK_POSITION_SETPOINT_FRAME_ID (0x2050100u)
#define SPARK_POSITION_SETPOINT_LENGTH (8u)

typedef struct __attribute__((packed)) {
    /**
     * By default, the unit is rotations, but it can be changed implicitly using
     * the Position Conversion Factor parameter
     *
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    float setpoint;

    /**
     * Range: -
     * Scale: 0.0009765923
     * Offset: 0
     */
    int16_t arbitrary_feedforward;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t pid_slot;

    /**
     * 0: Voltage, 1: Duty Cycle (-1 to 1)
     *
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t arbitrary_feedforward_units;
} spark_position_setpoint_t;