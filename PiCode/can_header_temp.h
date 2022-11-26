#include <stdint.h>

const uint8_t MESSAGE_LEN = 8;
const uint16_t HEARTBEAT_CONTROL_ID = 0x100;
const uint16_t HEARTBEAT_FRONT_ID = 0x200;
const uint16_t HEARTBEAT_REAR_ID = 0x300;

typedef struct {
    uint8_t brake_pos; // give 255 for full brake
    uint8_t throttle_pos;
    uint8_t steering_pos; // 128 as the middle 
    uint8_t blink_both; // 0 1
    uint8_t blink_left; // 0 1
    uint8_t blink_right; // 0 1
} heartbeat_control_data_t; // 10 Hz

typedef struct {
    uint16_t servo_current; // 2 bytes, raw data
    uint16_t servo_pos; // 2 bytes, raw data
} heartbeat_front_data_t;


typedef struct {
    uint8_t heartbeat;
} heartbeat_rear_data_t;
