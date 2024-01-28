#include "spi.h"
#include "log.h"

#include <stdio.h>

void Tx(spi_config_t spi_config, uint8_t * tx_buffer, uint8_t len){
    int spifd=spi_open("/dev/spidev2.0", spi_config);
    char msg[100];
    int ret = spi_write(spifd, tx_buffer, len);
    if (ret < 0) {
        sprintf(msg, "ERROR: failed to Tx spi msg: %s\n", tx_buffer);
    } else {
        sprintf(msg, "INFO: sent %s, to spidev2.0\n ",tx_buffer);
    }
    c_log(msg);
    spi_close(spifd); 
}

uint8_t * Rx(spi_config_t spi_config, uint8_t * rx_buffer, uint8_t len){
    int spifd=spi_open("/dev/spidev2.0", spi_config);
    int ret = spi_read(spifd, rx_buffer, len);
    if (ret < 0) {
        c_log("ERROR: failed to Rx spi msg\n");
    } else {
        char msg[100];
        sprintf(msg, "INFO: read %s, frpm spidev2.0\n ",rx_buffer);
        c_log(msg);
    }
    spi_close(spifd); 
}