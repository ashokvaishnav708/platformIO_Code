

#include <device.h>
#include <drivers/lora.h>
#include <errno.h>
#include <sys/util.h>
#include <zephyr.h>
#include <math.h>
#include <logging/log.h>
#include <drivers/hwinfo.h>
#include <drivers/gpio.h>

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
#define SEND_ACK_PKT            0x04
#define RECEIVE                 0x05

#define ALIVE                   0x08
#define ALIVE_ACK               0x09
#define START_RANGING           0x07
#define RE_RANGING_PKT          0x11
#define NONE                    0xFF
//

#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);


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


void main(void)
{
    // Hardware information gain
    uint8_t hwid[4];
    _ssize_t length = hwinfo_get_device_id(hwid, sizeof(hwid));
    uint32_t host_id = (hwid[0] << 24) | (hwid[1] << 16) | (hwid[2] << 8) | hwid[3];
    struct Coordinates dev_coords;

    //LoRa device from devicetree
    const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    struct lora_modem_config config;

    struct lora_ranging_params ranging_result;

    // Payload declaration
    struct Payload payload;
    uint8_t *payload_ptr;
    payload_ptr = &payload;

    int ret;
    int16_t rssi;
	int8_t snr;
    int len;
    //bool ranging_req = false;
    bool ranging_done = false;
    uint8_t operation = RECEIVE;


    LOG_INF("Device ID : %x", host_id);


    if (!device_is_ready(lora_dev)) {
		LOG_ERR("%s Device not ready", lora_dev->name);
		return;
	}

    if (!device_is_ready(led.port))
    {
        LOG_ERR("Led not ready");
        return;
    }
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    config.frequency = 2445000000;
	config.bandwidth = BW_1600;
	config.datarate = SF_7;
	config.preamble_len = 12;
	config.coding_rate = CR_4_5;
	config.tx_power = 10;
    config.tx = false;

    ret = lora_config(lora_dev, &config);
    if (ret < 0) {
        LOG_ERR("LoRa config failed");
        return;
    }

	while (1) {
        if (ranging_done) {
            ret = lora_config(lora_dev, &config);
            if (ret < 0) {
                LOG_ERR("LoRa config failed");
                return;
            }
            ranging_done = false;
        }
        
        switch (operation)
        {
            case RECEIVE:   LOG_INF("RECEIVE MODE.");
                            len = lora_recv(lora_dev, payload_ptr, MAX_DATA_LEN, K_FOREVER,
                                    &rssi, &snr);
                            if(len < 0)
                            {
                                operation = RECEIVE;
                                break;
                            }
                            operation = payload.operation;
                            break;
            
            case ALIVE: if (payload.host_id != host_id) {
                            operation = RECEIVE;
                            break;
                        }

                        gpio_pin_set_dt(&led, GPIO_ACTIVE_HIGH);

                        dev_coords = payload.coords;
                        payload.host_id = host_id;
                        payload.operation = ALIVE_ACK;

                        k_sleep(K_MSEC(20));
                        ret = lora_send(lora_dev, payload_ptr, sizeof(payload));
                        if (ret < 0){
                            k_sleep(K_MSEC(15));
                            operation = RECEIVE;
                            break;
                        }
                        k_sleep(K_MSEC(15));

                        gpio_pin_set_dt(&led, GPIO_ACTIVE_LOW);

                        operation = RECEIVE;
                        break;
            
            case START_RANGING: ret = lora_setup_ranging(lora_dev, &config, host_id, ROLE_RECEIVER);
                                if(ret != true) {
                                    LOG_ERR("LoRa config failed.");
                                    operation = RECEIVE;
                                }
                                //k_sleep(K_MSEC(10));
                                ret = lora_receive_ranging(lora_dev, &config, host_id, K_FOREVER);
                                
                                //k_sleep(K_MSEC(5));
                                ranging_result = lora_transmit_ranging(lora_dev, &config, host_id);
                                
                                if (ranging_result.status != false) payload.coords.flag = true;
                                else payload.coords.flag = false;
                                k_sleep(K_MSEC(10));
                                lora_config(lora_dev, &config);
        
                                payload.operation = RE_RANGING_PKT;
                                payload.coords.x = ranging_result.distance;
                                payload.coords.y = ranging_result.RSSIVal;
                                
                                lora_send(lora_dev, payload_ptr, sizeof(payload));
                                k_sleep(K_MSEC(20));

                                ranging_done = true;
                                //ranging_req = false;
                                operation = RECEIVE;
                                break;
        
            default: operation = RECEIVE;
        }
	} 
}

