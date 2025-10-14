#include "rs02.h"
//motor already includes jetgpio

#define SPI_MODE 0

int main(){

    int handle=spiOpen(0, 5000000, 0, 0, 8, 1, 1); //SPI 1

    uint8_t id = 0; //CAN_ID

    uint8_t rxpacket[11];
    
    uint8_t txpacket[11] = rs02_gen_can_header(handle, SPI_MODE, id, (uint16_t) RS02_ENABLE_DATA);
    
    //rs02_jetson_spi_tx(handle, , , )

    //print response packet - write a function for that

    motor_mit();
    
    spiClose();
}
