
#include <zephyr.h>
#include <drivers/gpio.h>

#define LED0_NODE DT_ALIAS(led0)

#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)

void main(void)
{

    const struct device *led;
    int ret;
    bool on = true;
    led = device_get_binding(LED0);

    if(led == NULL) return;

    ret = gpio_pin_configure(led, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
    if(ret < 0) return;

    while(1)
    {
        gpio_pin_set(led, PIN, (int)on);
        on = !on;
        k_msleep(1000);
    }
}
