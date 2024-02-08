#include "comm.h"
#include "log.h"

extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

#include <fcntl.h>    
#include <unistd.h>
#include <sys/ioctl.h>

int open_i2c() {
    int file = open("/dev/i2c-0", O_RDWR);
    if (file < 0) {
        log(std::string("ERROR: failed to open i2c bus"));
    }
    return file;
}

uint16_t read_i2c(int file, uint8_t dev_address, uint8_t reg) {
    if (ioctl(file, I2C_SLAVE, dev_address) < 0) {
        log(std::string("ERROR: failed to read from i2c bus address:" + dev_address));
    }
    uint16_t data = i2c_smbus_read_word_data(file, reg);
    log(std::string("INFO: read from i2c bus address:" + dev_address) + std::string("data: " + data));
    return data;
}

void write_i2c(int file, uint8_t dev_address, uint8_t reg, uint16_t data) {
    if (ioctl(file, I2C_SLAVE, dev_address) < 0) {
        log(std::string("ERROR: failed to write to i2c bus address:" + dev_address));
    }
    int ret = i2c_smbus_write_word_data(file, reg, data);
    if (ret < 0) {
        log(  std::string("ERROR: failed to write to i2c bus address:" + dev_address) 
            + std::string("reg: " + reg) 
            + std::string("data: " + data));
    } else {
        log(  std::string("INFO: wrote to i2c bus address:" + dev_address) 
            + std::string("reg: " + reg) 
            + std::string("data: " + data));
    }
}
