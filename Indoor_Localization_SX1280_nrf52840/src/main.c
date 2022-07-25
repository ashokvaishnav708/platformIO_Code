

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

//#define ANCHOR
#define MOBILE

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

struct __attribute__ ((__packed__)) Coordinates {
    bool flag;
    float x;
    float y;
};

/* 
************ Payload Format ************
DEVICE_ID | OPERATION | DATA_POINTER
*/

struct __attribute__ ((__packed__)) Payload {
    uint32_t host_id;
    uint8_t operation;
    struct Coordinates coords;
};

void main(void)
{
    const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);

    struct lora_modem_config config;

    struct Payload packet;
    uint8_t *payload_ptr;
    payload_ptr = &packet;

    int ret, len;
    int16_t rssi;
	int8_t snr;

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
    #ifdef MOBILE
    config.tx = true;
    #endif
    #ifdef ANCHOR
    config.tx = false;
    #endif

    
    while(1)
    {
        ret = lora_config(lora_dev, &config);
        if (ret < 0) {
            LOG_ERR("LoRa config failed");
            return;
        }


        #ifdef ANCHOR
        len = lora_recv(lora_dev, payload_ptr, MAX_DATA_LEN, K_FOREVER,
				&rssi, &snr);

        LOG_INF("Received data: %x %x , (RSSI:%ddBm, SNR:%ddBm)",
                packet.host_id, packet.operation, rssi, snr);
        #endif
        
        #ifdef MOBILE
        packet.host_id = 0x33d6f416;
        packet.operation = RANGING_INIT;
        packet.coords.flag = false;
        packet.coords.x = 0.0;
        packet.coords.y = 0.0;

        ret = lora_send(lora_dev, payload_ptr, sizeof(packet));
        #endif
        k_sleep(K_MSEC(1000));
    }
    
}

