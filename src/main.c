#include <stdio.h>

#include "rev/CANSparkFrames.h"
#include "CANAK60Frames.h"
#include "can.h"
#include "config.h"

#define ROBORIO_HEARTBEAT_FRAME_ID 0x01011840

int main() {
    float positions[NUM_MOTORS];
    while(1) {
        if(fread(positions, sizeof(float), NUM_MOTORS, stdin) != NUM_MOTORS)
            return 1;
        
        for(int i = 0; i < NUM_MOTORS; i++) {
            switch (configs[i].motor_type)
            {
            case SPARK_MAX: {
                // heartbeat
                can_command_header_t header = {configs[i].can_channel, 8};
                uint32_t arbitration = ROBORIO_HEARTBEAT_FRAME_ID;
                uint8_t data[8] = { 255 };

                fwrite(&header, sizeof(header), 1, stdout);
                fwrite(&arbitration, sizeof(arbitration), 1, stdout);
                fwrite(&data, sizeof(data), 1, stdout);
                
                // position set
                header = (can_command_header_t){configs[i].can_channel, SPARK_POSITION_SETPOINT_LENGTH};
                arbitration = configs[i].motor_id | SPARK_POSITION_SETPOINT_FRAME_ID;
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
                can_command_header_t header = {configs[i].can_channel, sizeof(ak60_position_setpoint_t)};
                uint32_t arbitration = CAN_AK60_MAKE_ARBITRATION(CAN_AK60_PACKET_SET_POS, configs[i].motor_id);
                ak60_position_setpoint_t data = (ak60_position_setpoint_t)(positions[i] * 10000.0);

                fwrite(&header, sizeof(header), 1, stdout);
                fwrite(&arbitration, sizeof(arbitration), 1, stdout);
                fwrite(&data, sizeof(data), 1, stdout);
                break;
            }
            }
        }
    }
}