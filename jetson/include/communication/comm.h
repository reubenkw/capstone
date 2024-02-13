#ifndef COMM_H
#define COMM_H

#include <cstdint>

#define MCU_1 0x01
#define MCU_ENCODER 0x10

#define DRIVE_MC 0x1
#define SERVO_MC 0x2

#define LIMIT_X 0
#define LIMIT_Y 0
#define LIMIT_Z 0

#define SYSFS_GPIO_DIR "/sys/class/gpio"

int open_i2c();
void read_i2c(int file, uint8_t mcu_addr, uint8_t * data, uint8_t len);
void write_i2c(int file, uint8_t mcu_addr, uint16_t data);

void initialize_gpio(unsigned int gpio);
int read_gpio(unsigned int gpio);
void write_gpio(unsigned int gpio, unsigned int val);
#endif //h