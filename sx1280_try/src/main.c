#include <zephyr.h>
//#include <drivers/lora.h>
#include <sx1280.h>


#define LORA_CONFIG_SX1280

#define SX1280_NODE DT_ALIAS(lora0)

void main(void)
{
    const struct device *sx1280_dev = DEVICE_DT_GET(SX1280_NODE);
    struct lora_modem_config config = {};

    //int ret;

    if(!device_is_ready(sx1280_dev))
    {
        printk("Device Not Ready");
        return;
    }

    config.frequency = 2480000000;
	config.bandwidth = Radiolorabandwidths_t BW_0400;
	config.datarate = SF_7;
	config.preamble_len = 12;
	config.coding_rate = CR_4_5;
	config.tx_power = 10;
	config.tx = true;

    while(1)
    {

    }
}
