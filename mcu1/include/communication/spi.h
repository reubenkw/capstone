#include <spidev_lib.h>

void Tx(spi_config_t spi_config, uint8_t * tx_buffer, uint8_t len);

uint8_t * Rx(spi_config_t spi_config, uint8_t * rx_buffer, uint8_t len);