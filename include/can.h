#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint8_t controller_num : 3;
    uint8_t data_len: 5; //should be larger to support max 64 bytes of CAN FD
} can_command_header_t;