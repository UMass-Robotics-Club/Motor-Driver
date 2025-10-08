#ifndef RS02_H
#define RS02_H

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
#define RS02_MIT_MODE_DATA {0x0F, 0xFF, 1, 2, 3, 4, 5, 6, 2, 0, 0}   
    /*0x 0F FF 01 02 03 04 05 06 02 00 00
    Documentation says this is an extended frame packet, 
    so it has an extra 18 bits, and needs its own generate_can_packet implementation.
    This will need to be accounted for in the firmware for the SPI-to-CAN board*/

#define RS02_ENABLE_DATA {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}
#define RS02_DISABLE_DATA {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}




int rs02_jetson_spi_tx(unsigned int spi_handle, uint8_t* packet, uint8_t* rxBuf);


uint8_t* rs02_gen_can_header(char channel,  char mode, char motor_id, char* data);


uint8_t* rs02_mit(float angle, float speed, float kp, float kd, float torque);


#endif //MOTOR_H