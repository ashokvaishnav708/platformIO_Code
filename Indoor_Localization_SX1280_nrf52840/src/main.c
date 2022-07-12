

#include <device.h>
#include <drivers/lora.h>
#include <errno.h>
#include <sys/util.h>
#include <zephyr.h>
#include <math.h>
#include <logging/log.h>

#define DEFAULT_RADIO_NODE DT_ALIAS(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
	     "No default LoRa radio specified in DT");

#define MAX_DATA_LEN 10
#define ROLE_SENDER 0x01
#define ROLE_RECEIVER 0x00

#define FIXED_ANCHOR
//#define MOBILE_DEVICE

#ifdef MOBILE_DEVICE
#define RANGING_SAMPLES 5
#endif

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL

typedef  enum
{
    RANGING_INIT           = 0x00,
    RANGING_DONE           = 0x01,
    RANGING_MODE_SLAVE     = 0x02,
    RANGIGN_MODE_MASTER    = 0x03,
    NONE                   = 0xFF,
}OperationType_t;

LOG_MODULE_REGISTER(Indoor_Localization);

const uint16_t TxtimeoutmS = 5000;
const uint16_t RxtimeoutmS = 0xFFFF;
const uint32_t device_address = 01;

struct packet {
    OperationType_t operation;
    uint32_t address;
};

void main(void)
{
    const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    const dev_id = rand();

    struct lora_modem_config config;

    struct packet data = {.operation = NONE, .address = dev_id};

    int ret;

    if (!device_is_ready(lora_dev)) {
		LOG_ERR("%s Device not ready", lora_dev->name);
		return;
	}

    config.frequency = 2445000000;
	config.bandwidth = BW_0800;
	config.datarate = SF_8;
	config.preamble_len = 12;
	config.coding_rate = CR_4_5;
	config.tx_power = 10;
    #ifdef MOBILE_DEVICE
    config.tx = true;
    #endif
    #ifdef FIXED_ANCHOR
    config.tx = false;
    #endif
    
}

