#include <zephyr.h>
#include <SX1280.h>
#include <drivers/spi.h>

#define NODE_SPI DT_NODELABEL(spi3)
#define NODE_GPIO0 DT_NODELABEL(gpio0)

/*
void make_transceive(const struct device *spi, struct spi_config* spi_cfg, uint8_t command, uint8_t rx_buf[], uint8_t rx_len) 
{
        uint8_t tx_buf[1] = {command};
        const struct spi_buf spi_buf_tx = {
                .buf = tx_buf,
                .len = sizeof(tx_buf)
        };
        struct spi_buf_set tx = {
                .buffers = &spi_buf_tx,
                .count = 1
        };

        struct spi_buf spi_buf_rx[] = {
                {
                        .buf = NULL,
                        .len = sizeof(tx_buf)
                },
                {
                        .buf = rx_buf,
                        .len = rx_len
                }
        };
        struct spi_buf_set rx = {
                .buffers = spi_buf_rx,
                .count = 2
        };
        spi_transceive(spi, spi_cfg, &tx, &rx);

        //spi_write(,,,);

}
*/

void main(void)
{
    int i = 0;
    struct SX1280 sx1280_dev;
    
    
    struct device *spi_dev = DEVICE_DT_GET(NODE_SPI);
    struct device *cs = DEVICE_DT_GET(NODE_CS);
    struct spi_cs_control spi_cs;
    spi_cs.gpio = 
    struct device 
    //struct device *sx1280 = device_get_binding("SPI_3");
    struct spi_config config = {};
    uint8_t rx_buf[32];

    config.frequency = 52000000;
    config.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
    config.cs = spi_cs;

    if (!device_is_ready(sx1280)) {
		printk("Device failed to configure");
		return;
	}

    while(1)
    {
        
        //printk("Received :%X", *((uint32_t*)rx_buf));
        k_sleep(K_MSEC(1000));
    }
}
