#include "motor.h"
#include "jetgpio.h"


int jetson_spi_tx(unsigned int spi_handle, char* packet, char* rxBuf){

    if (spi_handle < 0){
        printf("SPI was not setup correctly. Transfer cancelled.");
        return -1;
    }

    size_t packet_len = sizeof(packet) / sizeof(packet[0]);
    unsigned int len = (unsigned int)packet_len; //number of bytes to send

    spiXfer(spi_handle, packet, rxBuf, len);
    
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


char* motor_mit_mode(unsigned int spi_handle, char channel, char mode){

    //0x 0F FF 01 02 03 04 05 06 02 00 00
    //Documentation says this is an extended frame packet, 
    //so it has an extra 18 bits, and needs its own generate_can_packet implementation.
    
    //this will need to be accounted for in the firmware for the SPI-to-CAN board

    //Changes protocol to MIT

    char data = {0x0F, 0xFF, 1, 2, 3, 4, 5, 6, 2, 0, 0};

    

    char rxBuf[11];

    int spi_err = jetson_spi_tx(spi_handle, packet, rxBuf); //error code

    if (spi_err < 0){
        printf("SPI Error");
        return NULL;
    }

    return rxBuf;   
    //response frame gives us motor ID (11 bits) followed by the MCU ID
}


char* motor_enable(int spi_handle, char channel, char mode, char motor_id){

    unsigned char data[8];

    for (int i=0; i<7; i++){
        data[i] = 0xFF;
    }

    data[7] = 0xFC;

    char rxBuf[11];

    char* packet = gen_can_header_mit(channel, mode, motor_id, data);

    int spi_err = jetson_spi_tx(spi_handle, packet, rxBuf);

    if (spi_err < 0){
        printf("SPI Error");
        return NULL;
    }

    return rxBuf;
}

