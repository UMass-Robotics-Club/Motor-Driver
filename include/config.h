#include <stdint.h>

typedef enum {
    SPARK_MAX,
    AK60,
} MotorType;

typedef struct {
    uint8_t can_channel;
    MotorType motor_type;
    uint8_t motor_id;
} MotorConfig;

#define NUM_MOTORS 6

//TODO do this
MotorConfig configs[NUM_MOTORS] = {
    {1,SPARK_MAX,0},
};