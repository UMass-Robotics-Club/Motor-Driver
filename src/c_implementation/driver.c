#include <stdint.h>


#define P_MIN -12.57f
#define P_MAX 12.57f
#define V_MIN -44.0f
#define V_MAX 44.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -17.0f
#define T_MAX 17.0f


struct exCanIdInfo{
    uint32_t id:8;
    uint32_t data:16;
    uint32_t mode:5;
    uint32_t res:3;
};

struct can_receive_message_struct rxMsg;


struct can_transmit_message_struct {
    uint32_t tx_sfid;
    uint32_t tx_efid;
    uint8_t tx_ft;
    uint8_t tx_ff;
    uint8_t tx_dlen;
    uint8_t tx_data[8];
};

struct can_transmit_message_struct txMsg={
    .tx_sfid = 0, //standard frame id
    .tx_efid = 0xff, //Extended frame id
    .tx_ft = CAN_FT_DATA, //data in standard 11-bit format
    .tx_ff = CAN_FF_EXTENDED, //data in extended 29-bit format
    .tx_dlen = 8, 
    
};


#define txCanIdEx (*((struct exCanIdInfo*)&(txMsg.tx_efid)))
#define rxCanIdEx (*((struct exCanIdInfo*)&(rxMsg.rx_efid))) //Parses the extended frame id into a custom data structure

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
    can_txd();
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
    can_txd();
}



/*
#define can_txd() can_message_transmit(CAN0, &txMsg)
#define can_rxd() can_message_receive(CAN0, CAN_FIFO1, &rxMsg)
*/

