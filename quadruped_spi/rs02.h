#ifndef RS02_H
#define RS02_H

//Includes
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>



/*-----Format-----
Standard 11-bit ID section:
Bits [10:8] = Mode type
Bits [7:0] = Device ID

8-byte Data section:
Bytes 0-7 = Data
*/


/*--------------------------Constant Packets--------------------------*/
#define RS02_MIT_MODE_DATA (uint8_t[]){0x0F, 0xFF, 1, 2, 3, 4, 5, 6, 2, 0, 0}   
#define RS02_ENABLE_DATA (uint8_t[]){0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}
#define RS02_DISABLE_DATA (uint8_t[]){0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}
#define RSO2_SET_ZERO_POS_DATA (uint8_t[]){0xFF, 0XFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE} //works in non position mode 


/*-------------------------------Methods----------------------------------*/

/// @brief generates packet to send over SPI 
/// @param ext boolean extended frame or not
/// @param channel 1-6
/// @param mode part of arbitration
/// @param motor_id part of arbitration
/// @param data 
/// @returns spi tx packet
uint8_t* rs02_spi_tx_packet(int ext, uint8_t channel,  uint32_t arbitration, uint8_t* data);


/// @brief generic method to generate 8-byte data packet for MIT protocol
/// @param angle 
/// @param speed 
/// @param kp 
/// @param kd 
/// @param torque 
/// @returns 8-byte data packet
uint8_t* rs02_mit_data(float angle, float speed, float kp, float kd, float torque);


#endif //RS02_H