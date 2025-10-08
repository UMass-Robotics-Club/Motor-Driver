#ifndef MOTOR_H
#define MOTOR_H

//Includes
#include <stdlib.h>
#include <stdint.h>



/*-----Format-----
11-bit ID section:
Bits [10:8] = Mode type
Bits [7:0] = Device ID


8-byte Data section:
Bytes 0-7 = Data

*/

//Constant Packets
#define MIT_MODE_PACKET {0x0F, 0xFF, 1, 2, 3, 4, 5, 6, 2, 0, 0}   
    /*0x 0F FF 01 02 03 04 05 06 02 00 00
    Documentation says this is an extended frame packet, 
    so it has an extra 18 bits, and needs its own generate_can_packet implementation.
    This will need to be accounted for in the firmware for the SPI-to-CAN board*/

#define MOTOR_ENABLE_PACKET {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}
#define MOTOR_DISABLE_PACKET {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}




int jetson_spi_tx(unsigned int spi_handle, uint8_t* packet, uint8_t* rxBuf);


uint8_t* gen_can_header(char channel,  char mode, char motor_id, char* data);


uint8_t* motor_mit(float angle, float speed, float kp, float kd, float torque);


#endif //MOTOR_H