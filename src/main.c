#include <stdio.h>

#include "spark.h"
#include "ak60.h"
#include "can.h"
#include "config.h"

int main() {
    float positions[NUM_MOTORS];
    while(1) {
        fread(positions, sizeof(positions), 1, stdin);
        for(int i = 0; i < NUM_MOTORS; i++) {
            switch (configs[i].motor_type)
            {
            case SPARK_MAX: {
                can_command_header_t header = {configs[i].can_channel, SPARK_POSITION_SETPOINT_LENGTH};
                uint32_t arbitration = configs[i].motor_id | SPARK_POSITION_SETPOINT_FRAME_ID;
                spark_position_setpoint_t data = {
                    positions[i]/360.0, // setpoint (position is in degrees but we need rotations)
                    0, // arbitrary_feedforward (set to zero but may need to be changed)
                    0, // pid_slot
                    0, // arbitrary_feedforward_units (0 is voltage)
                };
                
                fwrite(&header, sizeof(header), 1, stdout);
                fwrite(&arbitration, sizeof(arbitration), 1, stdout);
                fwrite(&data, sizeof(data), 1, stdout);
                break;
            }
            case AK60: {
                can_command_header_t header = {configs[i].can_channel, AK60_POSITION_SETPOINT_LENGTH};
                uint32_t arbitration = configs[i].motor_id | AK60_POSITION_SETPOINT_FRAME_ID;
                int32_t data = positions[i] * 10000.0;

                fwrite(&header, sizeof(header), 1, stdout);
                fwrite(&arbitration, sizeof(arbitration), 1, stdout);
                fwrite(&data, sizeof(data), 1, stdout);
                break;
            }
            }
        }
    }
}