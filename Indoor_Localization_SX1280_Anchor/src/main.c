

#include <device.h>
#include <drivers/lora.h>
#include <errno.h>
#include <sys/util.h>
#include <zephyr.h>
#include <math.h>
#include <logging/log.h>
#include <drivers/hwinfo.h>

#define DEFAULT_RADIO_NODE DT_ALIAS(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
	     "No default LoRa radio specified in DT");

//#define ROLE_SENDER 0x01
#define ROLE_RECEIVER 0x00

#define MAX_DATA_LEN 255

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL

//Operations
#define RANGING_INIT            0x00
#define RANGING_DONE            0x01
#define RANGING_ACK             0x02
#define RANGIGN_MODE_MASTER     0x03
#define NONE                    0xFF
//

/* Hardware Addresses of currently available nrf52840 Anchors 
raspi03     48 d7 41 a7, 
raspi06     1c 1b 65 6e, 
raspi07     1d 5a 2a 62, 
raspi10     e7 32 e3 a5, 
raspi12     66 18 2c 94, 
raspi16     50 9a 12 7c,
*/

#define RASPI03     0x48d741a7
#define RASPI06     0x1c1b656e
#define RASPI07     0x1d5a2a62
#define RASPI10     0xe732e3a5
#define RASPI12     0x66182c94
#define RASPI16     0x509a127c

LOG_MODULE_REGISTER(Indoor_Localization_Anchor);

const uint16_t TxtimeoutmS = 5000;

const uint16_t RxtimeoutmS = 0xFFFF;  // 0xFFFF for forever
const uint32_t device_address = 02;


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


struct Coordinates get_dev_coordinates(uint32_t dev_id)
{
    struct Coordinates coords = {.x = -1, .y = -1};
    switch (dev_id)
    {
        case RASPI03: // raspi03
                break;
        case RASPI06: // raspi06
                break;
        case RASPI07: // raspi07
                break;
        case RASPI10: // raspi10
                break;
        case RASPI12: //raspi12
                break;
        case RASPI16: // raspi16
                break;
        default: break;
    }
    return coords;
}


void receive(const struct device *sx1280_dev, uint8_t *payload_ptr)
{
    int16_t rssi;
	int8_t snr;
    int len = -1;
    while(len < 0)
    {
        len = lora_recv(sx1280_dev, payload_ptr, MAX_DATA_LEN, K_FOREVER,
				&rssi, &snr);
        /*if (!(len < 0))
        {
            LOG_INF("Received data: %x %x , (RSSI:%ddBm, SNR:%ddBm)",
                payload.dev_id, payload.operation, rssi, snr);
        }*/
    }
}

void main(void)
{
    // Hardware information gain
    uint8_t hwid[4];
    _ssize_t length = hwinfo_get_device_id(hwid, sizeof(hwid));
    uint32_t host_id = (hwid[0] << 24) | (hwid[1] << 16) | (hwid[2] << 8) | hwid[3];
    const struct Coordinates dev_coords = get_dev_coordinates(host_id);

    //LoRa device from devicetree
    const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    struct lora_modem_config config;

    // Payload declaration
    struct Payload payload;
    uint8_t *payload_ptr;
    payload_ptr = &payload;
    
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
    config.tx = false;

    

	while (1) {
        // Setup LoRa Device
        ret = lora_config(lora_dev, &config);
        if (ret < 0) {
            LOG_ERR("LoRa config failed");
            return;
        }
		// Receive Mode
        recv_pkt:
        receive(lora_dev, payload_ptr);

        if (payload.operation != RANGING_INIT) goto recv_pkt;
        
        LOG_INF("Received data: %x %x",
            payload.dev_id, payload.operation);
        //LOG_INF("Coordinates : (%d, %d).", payload.coords.x, payload.coords.y);

        // Send back device address with ranging mode slave and it's coordinates

        payload.coords = dev_coords;
        payload.host_id = host_id;
        payload.operation = RANGING_ACK;

        ret = lora_send(lora_dev, payload_ptr, sizeof(payload));
        if(ret < 0)
        {
            LOG_ERR("Ranging Ack failed.");
            return;
        }
        ret = lora_setup_ranging(lora_dev, &config, host_id, ROLE_RECEIVER);
        if(ret != true) {
            LOG_ERR("LoRa config failed.");
            return;
        }
        if ( !(lora_receive_ranging(lora_dev, &config, host_id, RxtimeoutmS) ))
        {
            LOG_ERR("Ranging Timeout.");
        }
	} 
}

