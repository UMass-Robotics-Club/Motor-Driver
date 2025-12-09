#include "rs02.h"
//motor already includes jetgpio
#include <unistd.h>
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


int send_packet_rpi5(int handle, uint8_t* tx_buf, uint8_t* rx_buf, size_t len){
    int err;

    uint8_t rxdummy[2] = {0};
    
    for (int k=0; k<len; k++){ err = lgSpiXfer(handle, tx_buf+k, rxdummy, 1); }//send

    return err;
}



int main(){
    /* Jetson SPI setup
    int handle = spiOpen(SPI_CHAN, 5000000, SPI_MODE, 0, 8, 1, 1); //SPI 1
    */

    int chip = lgGpiochipOpen(0);

    if (chip >= 0) fprintf(stdout, "open successful\n");

    else{
        fprintf(stderr, "lgpio init failed\n");
        return -1;
    } 
    

    int handle;
    
    if ((handle = lgSpiOpen(0, 0, 1000000, 0)) < 0){
        printf("Error initializing GPIO");
    }

    uint8_t id = 0; //CAN_ID placeholder

    uint8_t rxpacket[14]={0}; 
    uint8_t* txpacket = rs02_spi_tx_packet(0, 1, id, RS02_ENABLE_DATA); //for output on can controller 1 with id 1 with enable command
    int err = send_packet_rpi5(handle, txpacket, rxpacket, 14);

    if (err >= 0){
        fprintf(stdout, "SPI transfer okay\n");
        fprintf(stdout, "TX: ");
        print_packet(txpacket, 14);
        fprintf(stdout, "RX: ");
        print_packet(rxpacket, 14);
    }

    else fprintf(stderr, "SPI transfer error: %d\n", err);
    
    lgSpiClose(handle);
    fprintf(stdout, "SPI closed\n");
    lgGpiochipClose(chip);
    return 0;
}
