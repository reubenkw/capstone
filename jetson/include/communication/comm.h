#ifndef COMM_H
#define COMM_H

#include <cstdint>

#define DRIVE_L_F 0x1000
#define DRIVE_L_B 0x1000
#define DRIVE_R_F 0x1000
#define DRIVE_R_B 0x1000

#define SERVO_X 0x1000
#define SERVO_Y 0x1000
#define SERVO_Z 0x1000

#define ENCODER_L_F 0x1000
#define ENCODER_L_B 0x1000
#define ENCODER_R_F 0x1000
#define ENCODER_R_B 0x1000

#define ENCODER_X 0x1000
#define ENCODER_Y 0x1000
#define ENCODER_Z 0x1000

int open_i2c(int dev_address);
uint16_t read_i2c(int file, uint8_t dev_address, uint8_t reg);
void write_i2c(int file, uint8_t dev_address, uint8_t reg, uint16_t data);

#endif //h