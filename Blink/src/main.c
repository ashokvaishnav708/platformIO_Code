
#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>

LOG_MODULE_REGISTER(sample_try);


#define BUTTON_NODE DT_NODELABEL(button0)
#define LED_NODE DT_NODELABEL(led0)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);


void main(void)
{
    int ret;

    if(!device_is_ready(led.port)) return;
    if(!device_is_ready(button.port)) return;

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
    if(ret < 0) return;

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if(ret < 0) return;

    while(1)
    {
        LOG_INF("Hello World.");
        k_sleep(K_MSEC(1000));
    }

}
