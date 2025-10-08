#include "motor.h"
//motor already includes jetgpio

int main(){

    int handle=spiOpen(0, 5000000, 0, 0, 8, 1, 1); //SPI 1

    motor_mit();

    

    uint8_t txpacket[11]; 
    uint8_t rxpacket[11];
    
    
    spiClose();
}
