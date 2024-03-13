#ifndef SPI_H
#define SPI_H

#include "driver/spi_common.h"
#include "driver/spi_master.h"

#define SPI_HOST SPI2_HOST

#define WRITE 0
#define READ 1

#define SPI_INIT_ERROR 0
#define SPI_TX_ERROR 1
#define SPI_RX_ERROR 2

#define STAT_OK 0x0
#define SPI_TX_ERR 0x70
#define SPI_RX_ERR 0x80

extern spi_device_handle_t spi_mc_dc_handle;

void init_spi();
uint8_t read_spi(spi_device_handle_t device, uint addr);
void write_spi(spi_device_handle_t device, uint addr, uint8_t tx_data);
void clear_errors();

#endif // h
