#include "rs02.h"
//motor already includes jetgpio
#include <pigpio.h>

#define SPI_CHAN 0
#define SPI_MODE 0


int print_packet(uint8_t* packet, size_t len){

    for(int i=0; i<len; i++){
        printf("%02X ", packet[i]);
    }

    printf("\n");
    return 0;
}

int main(){

    /* Jetson SPI setup
    int handle = spiOpen(SPI_CHAN, 5000000, SPI_MODE, 0, 8, 1, 1); //SPI 1
    */

    /*RPI 5 SPI setup*/
    if (gpioInitialise() < 0){
        fprintf(stderr, "pigpio failed to start.");
        return 1;
    }

    uint32_t spiflags = 0; 
    spiflags |= (14 << 10); //read 14 bytes before switching MOSI to MISO
    int handle = spiOpen(SPI_CHAN, 500000, spiflags); //SPI 1



    uint8_t id = 0; //CAN_ID placeholder
    uint8_t rxpacket[11]; //response packet
    uint8_t* txpacket = rs02_spi_tx_packet(0, SPI_MODE, id, RS02_ENABLE_DATA);
    
    rs02_jetson_spi_tx(handle, txpacket, rxpacket);

    print_packet(rxpacket, sizeof(rxpacket));
    
    spiClose(handle);
    gpioTerminate();
    return 0;
}
