#include <stdint.h>
#ifndef MOTOR_H
#define MOTOR_H


//Constants
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



//Structs
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


//Macros
#define txCanIdEx (*((struct exCanIdInfo*)&(txMsg.tx_efid)))
#define rxCanIdEx (*((struct exCanIdInfo*)&(rxMsg.rx_efid))) //Parses the extended frame id into a custom data structure


//Functions
int float_to_uint(float x, float x_min, float x_max, int bits);


void motor_enable(uint8_t id, uint16_t master_id);


void motor_controlmode(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd);



/*
#define can_txd() can_message_transmit(CAN0, &txMsg)
#define can_rxd() can_message_receive(CAN0, CAN_FIFO1, &rxMsg)
*/

#endif //MOTOR_H
