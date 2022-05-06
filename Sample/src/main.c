
#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <sx1280.h>


#define SLEEP_TIMEMS 500

#define BUTTON_NODE DT_NODELABEL(button0)
#define LED_NODE DT_NODELABEL(led0)
#define SPI_NODE DT_NODELABEL(spi0)
#define SPI_CS_NODE DT_NODELABEL(led1)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static const struct gpio_dt_spec button_led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
static const struct spi_cs_control cs_pin = SPI_CS_CONTROL_PTR_DT(DT_NODELABEL(led1), 0);

void transceive(const struct device *spi, struct spi_config* spi_cfg, uint8_t command, uint8_t rx_buf[], uint8_t rx_len) 
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
}


void main(void)
{
    int ret;
    struct device *spi;
    //struct device *button_led;
    //struct device *button;
    //struct device *cs_pin;

    struct spi_config spi_cfg = {};

    //spi = device_get_binding(DT_LABEL(SPI_NODE));
    //button_led = device_get_binding(DT_LABEL(LED_NODE));
    //button = device_get_binding(DT_LABEL(BUTTON_NODE));
    //cs_pin = device_get_binding(DT_LABEL(SPI_CS_NODE));

    if(!device_is_ready(button_led.port)) return;
    if(!device_is_ready(button.port)) return;
    //if(!device_is_ready(spi)) return;
    //if(!device_is_ready(cs_pin)) return;
    

    ret = gpio_pin_configure_dt(&button_led, GPIO_OUTPUT);
    if(ret < 0) return;

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if(ret < 0) return;

    //ret = gpio_pin_configure_dt(&cs_pin, GPIO_ACTIVE_LOW);
    //if(ret < 0) return;

    //SPI parameters configuration
    spi_cfg.frequency = 52000000;
    spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER;
    //spi_cfg.cs = &cs_pin;

    while(1)
    {
        //int val = gpio_pin_get_dt(button);
        //if(val >= 0)
        //{
        gpio_pin_set_dt(&button_led, true);
        printk("\nLED : ON");
        //}
        k_msleep(SLEEP_TIMEMS);
        gpio_pin_set_dt(&button_led, false);
        printk("\nLED : OFF");
        k_msleep(SLEEP_TIMEMS);
    }   

}
