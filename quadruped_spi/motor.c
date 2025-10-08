#include "motor.h"
#include "jetgpio.h"


int jetson_spi_tx(unsigned int spi_handle, uint8_t* packet, uint8_t* rxBuf){

    if (spi_handle < 0){
        printf("SPI was not setup correctly. Transfer cancelled.");
        return -1;
    }

    size_t packet_len = sizeof(packet) / sizeof(packet[0]);
    unsigned int len = (unsigned int)packet_len; //number of bytes to send

    spiXfer(spi_handle, packet, rxBuf, len);
    
    return 0;
}


uint8_t* gen_can_header(char channel,  char mode, char motor_id, char* data){
/*Format: [channel: 1 byte | id: 2 bytes | data: 8 bytes] = 11 bytes
  
  ID is separated into two parts: the mode of operation of the motor (byte 1 bits 0,1,2. 
  Other bits are don't care and should be masked out) followed by the CAN ID of the device to control (byte 2)
  
  Data is just 8 bytes of straight data. (bytes 3 to 10)

  The length of the CAN message is 11 bits + 8 bytes
  The additional channel byte should be used by the Mega CAN board to choose a CAN channel to communicate on.
*/
    
    uint8_t out[11];
    
    //Channel
    out[0] = channel; //byte 0
    
    //ID
    out[1] = mode; //byte 1
    out[2] = motor_id; //byte 2
    
    //Data
    for (int j=0; j<8; j++){ //bytes 3 to 10
        out[j+3] = data[j]; //offset of 3
    }

    return out;
}


uint8_t* motor_mit(uint16_t angle, uint16_t speed, uint16_t kp, uint16_t kd, uint16_t torque){
    
    /*Ranges
    --------
    angle:  0-65535 (-12.57 rad ~ 12.57 rad)
    speed: 0-4096 (-44 rad/s ~ 44 rad/s)
    kp: 0-4096 (0 ~ 500)
    kd: 0-4096 (0 ~ 5)
    torque: 0-4096 (-17 N·m ~ 17 N·m)*/


    /*MIT dynamic parameters protocol

    angle: Bytes 0 and 1
    target speed: Byte 2 and Byte 3 [7:4]
    kp: Byte 3[3:0] and Byte 4[8:0]
    kd: Byte 5[8:0] and Byte 6[7:4]
    torque: Byte 6[3:0] and Byte 7[8:0]
    */

    


    //ADD RANGE CHECKS HERE
    //-------------------------


    //-------------------------


    uint8_t data[11];

    //Angle
    data[0] = (angle >> 8) & 0xFF;  //high 8 bits
    data[1] = angle & 0xFF;  //low 8 bits


    //Target speed and kp
    data[2] = (speed >> 4) & 0xFF; //high 8 bits
    
    uint8_t speed_low = (speed >> 8) & 0x0F; //low 4 bits
    uint8_t kp_high = (kp >> 8) & 0x0F; //high 4 bits

    data[3] = ((speed_low << 4) | kp_high); //packing kp and speed bits together
    
    data[4] = (kp >> 4) & 0xFF; //low 8 bits

    
    //kd and torque
    data[5] = (kd >> 4) & 0xFF; //8 bits

    uint8_t kd_low = (kd >> 8) & 0x0F; //4 bits
    uint8_t t_high = (torque >> 8) & 0x0F; //4 bits

    data[6] = ((kd_low << 4) | t_high); //packing 

    data[7] = (torque >> 4) & 0xFF;
    
    return data;

}