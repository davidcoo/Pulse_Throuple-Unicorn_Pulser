#include <types.h>

typedef struct {
    uint8_t len;
    uint16_t id;
} can_header_t;

const uint8_t HEARTBEAT_CONTROL_LEN = 6;
const uint16_t HEARTBEAT_CONTROL_ID = 0x100;

typedef struct {
    uint16_t steering_pos;
    uint16_t brake_pos;
    uint16_t accel_pos;
} heartbeat_control_data_t;

typedef enum {
    BLINK_OFF = 0;
    BLINK_LEFT = 1;
    BLINK_RIGHT = 2;
} blinker_state_e;

typedef struct {
    blinker_state_e blinker_state;
} blinker_data_t;

typedef enum {
    COLLISION_NONE = 0;
    COMMISION_20 = 1;
    COLLISION_10 = 2;
} collision_state_e;

typedef enum {
    COLLISION_FRONT = 0;
    COLLISION_LEFT = 1;
    COLLISION_RIGHT = 2;
} collision_dir_e;

typedef struct {
    collision_state_e collision_state;
    collision_dir_e collision_dir;
} collision_msg_data_t;

typedef struct {
    uint16_t servo_current;
    uint16_t servo_pos;
} heartbeat_front_data_t;

typedef struct {
} heartbeat_rear_data_t;
