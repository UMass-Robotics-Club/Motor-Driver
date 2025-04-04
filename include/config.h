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
    {0,SPARK_MAX,0}, // shoulder pitch left
    {1,SPARK_MAX,0}, // shoulder pitch right
    {0,AK60,0}, // bicep left
    {1,AK60,0}, // bicep right
    {0,SPARK_MAX,0}, // shoulder roll left
    {1,SPARK_MAX,0}, // shoulder roll right
};