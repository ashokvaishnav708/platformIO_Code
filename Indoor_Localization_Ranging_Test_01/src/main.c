#include <device.h>
#include <drivers/lora.h>
#include <errno.h>
#include <sys/util.h>
#include <zephyr.h>
#include <math.h>
#include <logging/log.h>
#include <drivers/hwinfo.h>
#include <drivers/gpio.h>
#include <drivers/hwinfo.h>

#define DEFAULT_RADIO_NODE DT_ALIAS(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
             "No default LoRa radio specified in DT");

#define ROLE_SENDER 0x01
#define ROLE_RECEIVER 0x00

#define RASPI03 0x48d741a7
#define RASPI06 0x1c1b656e
#define RASPI07 0x1d5a2a62
#define RASPI10 0xe732e3a5
#define RASPI12 0x66182c94
#define RASPI16 0x509a127c

#define LOCAL31 0x33d6f416
#define LOCAL15 0x2ebbf779

#define MAX_DATA_LEN 255

// Receiver 31 --  33 d6 f4 16
// Transmitter 15 -- 2e bb f7 79

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>

LOG_MODULE_REGISTER(sx1280_ranging);

void main(void)
{

    uint32_t RangignAddress = 0x48d741a6;
    uint8_t hwid[4];
    ssize_t length;
    length = hwinfo_get_device_id(hwid, sizeof(hwid));
    uint32_t host_id = (hwid[0] << 24) | (hwid[1] << 16) | (hwid[2] << 8) | hwid[3];

    const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    struct lora_modem_config config;
    enum lora_datarate datarate[6] = {SF_5, SF_6, SF_7, SF_8, SF_9, SF_10};
    enum lora_signal_bandwidth bandwidth[3] = {BW_0400, BW_0800, BW_1600};
    struct lora_ranging_params rangingResult;

    int bw_index = 2;
    int dr_index = 5;

    bool sender = false;
    bool receiver = false;

    if ((host_id == RASPI06) || (host_id == LOCAL31))
    {
        sender = true;
    }

    if ((host_id == RASPI10) || (host_id == LOCAL15))
    {
        receiver = true;
    }

    if (!device_is_ready(lora_dev))
    {
        LOG_ERR("%s Device not ready.", lora_dev->name);
        return;
    }

    config.frequency = 2450000000;
    config.bandwidth = bandwidth[bw_index];
    config.datarate = datarate[dr_index];
    config.preamble_len = 12;
    config.coding_rate = CR_4_5;
    config.tx_power = 10;
    if (sender)
    {
        config.tx = true;

        lora_setup_ranging(lora_dev, &config, RangignAddress, ROLE_SENDER);
        k_sleep(K_MSEC(5000));
    }
    else if (receiver)
    {
        config.tx = false;

        lora_setup_ranging(lora_dev, &config, RangignAddress, ROLE_RECEIVER);
    }
    else
        while (1)
            ;

    while (1)
    {

        if (sender)
        {
            rangingResult = lora_transmit_ranging(lora_dev, &config, RangignAddress);
            if (rangingResult.status != false)
            {
                LOG_INF("SF: %d BW: %d RSSI: %d Dist: %d",
                        datarate[dr_index], bandwidth[bw_index], rangingResult.RSSIVal, rangingResult.distance);
            }
            else
            {
                LOG_ERR("Ranging Failed.");
            }
            k_sleep(K_MSEC(50));
        }
        else if (receiver)
        {
            lora_receive_ranging(lora_dev, &config, RangignAddress, K_FOREVER);
        }
    }
}
