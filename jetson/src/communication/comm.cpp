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

#define READ 0
#define WRITE 1

void read_i2c(int file, uint8_t mcu_addr, uint8_t * data, uint8_t len) {
    if (ioctl(file, I2C_SLAVE, mcu_addr) < 0) {
        log(std::string("ERROR: failed to read from i2c bus address:" + mcu_addr));
    }
    i2c_smbus_read_i2c_block_data(file, READ, len, data);
    log(std::string("INFO: read from i2c bus address:" + mcu_addr) + std::string("data: "));

    for (int i = 0; i < len; i++){
        log(std::to_string(data[i]));
    }
}

void write_i2c(int file, uint8_t mcu_addr, uint16_t data){
    if (ioctl(file, I2C_SLAVE, mcu_addr) < 0) {
        log(std::string("ERROR: failed to write to i2c bus address:" + mcu_addr));
    }
    int ret = i2c_smbus_write_word_data(file, WRITE, data);
    if (ret < 0) {
        log(  std::string("ERROR: failed to write to i2c bus address:" + mcu_addr) 
            + std::string("cmd: " + WRITE) 
            + std::string("data: " + data));
    } else {
        log(  std::string("INFO: wrote to i2c bus address:" + mcu_addr) 
            + std::string("cmd: " + WRITE) 
            + std::string("data: " + data));
    }
}
