#ifndef COMM_H
#define COMM_H

#include <cstdint>

#define MCU_M 0x42
#define MCU_E 0x10

#define DRIVE_MC 0x1
#define SERVO_MC 0x2
#define LIMIT_SWITCHES 0x3

typedef enum {
  S_PROCESSING_CMD,
  S_ACTION_COMPLETE,
  S_ACTION_ENDED_W_LIMIT,
} mcu_e_status_t;

typedef enum {
    CMD_WRITE_STATUS,
    CMD_MOVE_AXIS,  // 3 extra data bytes
    CMD_RESET,
    CMD_POLLINATE,
} jetson_2_mcu_e_commands_t;

int open_i2c();
void read_i2c(int file, uint8_t mcu_addr, uint8_t command, uint8_t * data, uint8_t len);
void write_i2c(int file, uint8_t mcu_addr, uint8_t command, uint8_t * data, uint8_t len);

#endif //h
