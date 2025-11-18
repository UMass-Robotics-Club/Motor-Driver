#include "rs02.h"
//motor already includes jetgpio
#include <lgpio.h>

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

    /*RPI 5 LGPIO SPI setup*/
    int chip = lgGPIOchipOpen(0);

    if (chip >= 0) fprintf (stdout, "open successful\n");

    else fprintf(stderr, "lgpio init failed\n");
    int handle = lgSpiOpen(0, 0, 500000, 0);


    uint8_t id = 0; //CAN_ID placeholder
    uint8_t rxpacket[11]; //response packet
    uint8_t* txpacket = rs02_spi_tx_packet(0, SPI_MODE, id, RS02_ENABLE_DATA);
    
    //rs02_jetson_spi_tx(handle, txpacket, rxpacket);

    if (lgSpiXfer(handle, txpacket, rxpacket, sizeof(txpacket)) > 0){
        fprintf(stdout, "SPI transfer okay\n");
        print_packet(rxpacket, sizeof(rxpacket));
    }

    else fprintf(stderr, "SPI transfer error\n");

    
    lgSpiClose(handle);
    lgGpiochipClose(chip);
    return 0;
}
