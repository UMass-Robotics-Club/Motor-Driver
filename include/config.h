#include <stdint.h>

typedef enum {
    SPARK_MAX,
    AK60,
} motor_type_t;

typedef struct {
    uint8_t can_channel;
    motor_type_t motor_type;
    uint8_t motor_id;
} motor_config_t;

#define NUM_MOTORS 6

//////////////////////////
// Configure Motors
//////////////////////////
motor_config_t configs[NUM_MOTORS] = {
    {0,SPARK_MAX,0},
    {1,SPARK_MAX,0},
    {2,SPARK_MAX,0},
    {3,SPARK_MAX,0},
    {4,SPARK_MAX,0},
    {5,SPARK_MAX,0},
};