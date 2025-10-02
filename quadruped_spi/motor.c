#include "motor.h"
#include "jetgpio.h"


int jetson_spi_setup(int spi_channel){
    unsigned int handle = spiOpen(spi_channel, 5000000, 0, 0, 8, 1, 1);

    if (handle < 0){
        return -1; //spi does not setup correctly
    }

    return handle;
}

int jetson_spi_txd(unsigned int spi_handle, char* packet, char* rxBuf){
    
    size_t packet_len = sizeof(packet) / sizeof(packet[0]);
    unsigned int len = (unsigned int)packet_len;

    if (spi_handle < 0){
        printf("SPI was not setup correctly. Transfer cancelled.");
        return -1;
    }

    for (int i=0; i<len; i++){
        spiXfer(spi_handle, packet, rxBuf, len);
    }

    return 0;
}

char* gen_can_header_mit(char channel,  char mode, char motor_id, char* data){
/*Format: [channel: 1 byte | id: 2 bytes | data: 8 bytes] = 11 bytes
  
  ID is separated into two parts: the mode of operation of the motor (byte 1 bits 0,1,2. 
  Other bits are don't care and should be masked out) followed by the CAN ID of the device to control (byte 2)
  
  Data is just 8 bytes of straight data. (bytes 3 to 10)

  The length of the CAN message is 11 bits + 8 bytes
  The additional channel byte should be used by the Mega CAN board to choose a CAN channel to communicate on.
*/
    
    unsigned char out[11];
    
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



int motor_enable(char channel, char mode, char motor_id){

    unsigned char data[8];

    for (int i=0; i<7; i++){
        data[i] = 0xFF;
    }

    data[7] = 0xFC;

    gen_can_header_mit(channel, mode, motor_id, data);

    return 0;
}

