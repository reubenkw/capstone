#include "spi.h"
#include "log.h"

#include <stdio.h>
#include <string.h>

int main(int argc, char** argv)
{
    c_log("hello world\n");

    spi_config_t spi_config;
    uint8_t tx_buffer[32];
    uint8_t rx_buffer[32];

    spi_config.mode=0;
    spi_config.speed=1000000;
    spi_config.delay=0;
    spi_config.bits_per_word=8;

    memset(tx_buffer,0,32);
    memset(rx_buffer,0,32);
    sprintf(tx_buffer,"transmitting hello world");

    Tx(spi_config, tx_buffer, strlen(tx_buffer));
    Rx(spi_config, rx_buffer, strlen(rx_buffer));

	return 0;
}