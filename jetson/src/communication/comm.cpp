#include "comm.h"

extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

#include <fcntl.h>    
#include <unistd.h>
#include <sys/ioctl.h>

int open_i2c(int address) {
    int file = open("/dev/i2c-0", O_RDWR);
    if (file < 0) {
        // error
    }
    return file;
}

uint16_t read_i2c(int file, uint8_t dev_address, uint8_t reg) {
    if (ioctl(file, I2C_SLAVE, dev_address) < 0) {
        // error
    }
    return i2c_smbus_read_word_data(file, reg);
}

void write_i2c(int file, uint8_t dev_address, uint8_t reg, uint16_t data) {
    if (ioctl(file, I2C_SLAVE, dev_address) < 0) {
        // error
    }
    int ret = i2c_smbus_write_word_data(file, reg, 0x1ffc);
    if (ret < 0) {
        // error
    }
}
