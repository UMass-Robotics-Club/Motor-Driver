#ifndef MOTOR_H
#define MOTOR_H

//Includes
#include <stdlib.h>



/*-----Format-----
11-bit ID section:
Bits [10:8] = Mode type
Bits [7:0] = Device ID


8-byte Data section:
Bytes 0-7 = Data

*/

int jetson_spi_tx(unsigned int spi_handle, char* packet, char* rxBuf);

char* gen_can_header_mit(char channel, char mode, char motor_id, char* data);

char* motor_enable(int spi_handle, char channel, char mode, char motor_id);


#endif //MOTOR_H