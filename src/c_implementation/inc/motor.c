#include "motor.h"

struct can_transmit_message_struct txMsg={
    .tx_sfid = 0, //standard frame id
    .tx_efid = 0xff, //Extended frame id
    //.tx_ft = CAN_FT_DATA is data in standard 11-bit format
    //.tx_ff = CAN_FF_EXTENDED is data in extended 29-bit format
    .tx_dlen = 8, 
    
};


int spi_txd(){
    //#define can_txd() can_message_transmit(CAN0, &txMsg)



}

/*
#define can_rxd() can_message_receive(CAN0, CAN_FIFO1, &rxMsg)
*/




int float_to_uint(float x, float x_min, float x_max, int bits){

    float span = x_max - x_min;
    float offset = x_min;

    if(x > x_max) x=x_max;

    else if(x < x_min) x= x_min;

    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}



void motor_enable(uint8_t id, uint16_t master_id)
{
    txCanIdEx.mode = 3;
    txCanIdEx.id = id;
    txCanIdEx.res = 0;
    txCanIdEx.data = master_id;
    txMsg.tx_dlen = 8;
    txCanIdEx.data = 0;

    //can_txd();
    spi_txd();
}



void motor_controlmode(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd)
{
    txCanIdEx.mode = 1;
    txCanIdEx.id = id;
    txCanIdEx.res = 0;
    txCanIdEx.data = float_to_uint(torque,T_MIN,T_MAX,16);
    txMsg.tx_dlen = 8;
    txMsg.tx_data[0]=float_to_uint(MechPosition,P_MIN,P_MAX,16)>>8;
    txMsg.tx_data[1]=float_to_uint(MechPosition,P_MIN,P_MAX,16);
    txMsg.tx_data[2]=float_to_uint(speed,V_MIN,V_MAX,16)>>8;
    txMsg.tx_data[3]=float_to_uint(speed,V_MIN,V_MAX,16);
    txMsg.tx_data[4]=float_to_uint(kp,KP_MIN,KP_MAX,16)>>8;
    txMsg.tx_data[5]=float_to_uint(kp,KP_MIN,KP_MAX,16);
    txMsg.tx_data[6]=float_to_uint(kd,KD_MIN,KD_MAX,16)>>8;
    txMsg.tx_data[7]=float_to_uint(kd,KD_MIN,KD_MAX,16);

    //can_txd();
    spi_txd();
}



void motor_reset(uint8_t id, uint16_t master_id)
{
    txCanIdEx.mode = 4;
    txCanIdEx.id = id;
    txCanIdEx.res = 0;
    txCanIdEx.data = master_id;
    txMsg.tx_dlen = 8;
    
    for(uint8_t i=0; i<8; i++){ txMsg.tx_data[i]=0; }

    //can_txd();
    spi_txd();
}


//this function needs comments
uint8_t runmode;
uint16_t index;
void motor_modechange(uint8_t id, uint16_t master_id)
{
    txCanIdEx.mode = 0x12;
    txCanIdEx.id = id;
    txCanIdEx.res = 0;
    txCanIdEx.data = master_id;
    txMsg.tx_dlen = 8;
    
    for(uint8_t i=0; i<8; i++){ txMsg.tx_data[i]=0; }

    memcpy(&txMsg.tx_data[0],&index,2);
    memcpy(&txMsg.tx_data[4],&runmode, 1);
    
    //can_txd();
    spi_txd();
}



//this function needs comments
float ref;
void motor_write(uint8_t id, uint16_t master_id)
{
    txCanIdEx.mode = 0x12;
    txCanIdEx.id = id;
    txCanIdEx.res = 0;
    txCanIdEx.data = master_id;
    txMsg.tx_dlen = 8;

    for(uint8_t i=0; i<8; i++){ txMsg.tx_data[i]=0; }
    
    memcpy(&txMsg.tx_data[0],&index,2);
    memcpy(&txMsg.tx_data[4],&ref,4);

    //can_txd();
    spi_txd();
}

