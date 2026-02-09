#include "rs02.h"
//rs02 includes jetgpio
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#define SPI_CHAN 0
#define SPI_MODE 0


int print_packet(uint8_t* packet, size_t len){

    for(int i=0; i<len; i++){
        printf("%02X ", packet[i]);
    }

    printf("\n");
    return 0;
}


int send_packet_rpi4(int handle, uint8_t* data_buf, size_t len){
    int err;
    
    for (int k=0; k<len; k++){err = wiringPiSPIDataRW(SPI_CHAN, data_buf+k, 1); }//send and receive (wiringpi uses the same buffer)

    return err;
}



int main(){
    /* ---------------Jetson SPI setup------------------
    int handle = spiOpen(SPI_CHAN, 5000000, SPI_MODE, 0, 8, 1, 1); //SPI 1
    ---------------------------------------------------*/

    /*PI4(WiringPI) Setup*/

    int spiFd = 0;
    if ((spiFd = wiringPiSPISetupMode(SPI_CHAN, 1000000, SPI_MODE)) < 0){
        fprintf(stderr, "Can't open the SPI bus");
    }


    uint8_t id = 0; //CAN_ID placeholder


    uint8_t* data_buffer = rs02_spi_tx_packet(0, 1, id, RS02_ENABLE_DATA); //for output on can controller 1 with id 1 with enable command
    //data_buffer = rs02_spi_tx_packet(1, 1, id, RS02_MIT_MODE_DATA);
    
    //int err = send_packet_rpi4(handle, data_buffer, 14);
    fprintf(stdout, "TX: ");
    print_packet(data_buffer, 14);
    int err = send_packet_rpi4(SPI_CHAN, data_buffer, 14);


    if (err >= 0){
        fprintf(stdout, "SPI transfer successful\n");
        fprintf(stdout, "RX unavailable\n");
        //print_packet(data_buffer, 14);
    }

    else fprintf(stderr, "SPI transfer error: %d\n", err);
    
    wiringPiSPIClose(SPI_CHAN);
    fprintf(stdout, "SPI closed\n");
    return 0;
}
