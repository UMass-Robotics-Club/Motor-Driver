#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint8_t controller_num : 3;
    uint8_t data_len: 5;
} can_command_header_t;