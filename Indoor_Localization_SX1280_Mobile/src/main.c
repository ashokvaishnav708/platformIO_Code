

#include <device.h>
#include <drivers/lora.h>
#include <errno.h>
#include <sys/util.h>
#include <zephyr.h>
#include <math.h>
#include <logging/log.h>
#include <drivers/hwinfo.h>
#include <stdlib.h>

#define DEFAULT_RADIO_NODE DT_ALIAS(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
	     "No default LoRa radio specified in DT");

#define ROLE_SENDER 0x01

#define MAX_DATA_LEN 249

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL

//Operations
#define RANGING_INIT            0x00
#define RANGING_DONE            0x01
#define RANGING_ACK             0x02
#define RANGIGN_MODE_MASTER     0x03
#define NONE                    0xFF
//

LOG_MODULE_REGISTER(Indoor_Localization_Mobile);

const uint16_t TxtimeoutmS = 5000;
const uint16_t RxtimeoutmS = 0xFFFF;
const uint32_t device_address = 01;




struct __attribute__ ((__packed__)) Coordinates {
    bool flag;      //  Validated Coordinates if TRUE else not validated
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

struct Anchor {
    uint32_t host_id;
    struct Coordinates coords;
    struct Anchor *next;
};

/* ********* Queue Operations ********** */

struct Anchor *front = NULL;
struct Anchor *rear = NULL;

void add_anchor(struct Anchor anchor)
{
    struct Anchor *n_anchor = malloc(sizeof(struct Anchor));
    n_anchor->coords = anchor.coords;
    n_anchor->host_id = anchor.host_id;
    n_anchor->next = anchor.next;

    if(rear == NULL)
    {
        front = n_anchor;
        rear = n_anchor;
    }
    else {
        rear->next = n_anchor;
        rear = rear->next;
    }
}

bool remove_anchor()
{
    if (front == NULL)
    {
        return false;
    } 
    struct Anchor *temp_anchor;
    temp_anchor = front;
    front = front->next;
    free(temp_anchor);
    return true;
}

int get_anchor_count()
{
    int count = 0;
    struct Anchor *temp_anchor;
    temp_anchor = front;
    while(temp_anchor != NULL)
    {
        count++;
        temp_anchor = temp_anchor->next;
    }

    return count;
}


bool broadcast_ranging_req(const struct device *sx1280_dev)
{
    int ret;
    uint8_t *payload_ptr;
    /*
    uint8_t data[MAX_DATA_LEN] = {RANGING_INIT, 
                                ((device_address >> 24u ) & 0xFFu),
                                ((device_address >> 16u ) & 0xFFu),
                                ((device_address >> 8u ) & 0xFFu),
                                (device_address & 0xFFu), };
    */

    
    struct Payload payload = {  .host_id = 01, 
                                .operation = RANGING_INIT,
                                .coords.flag = true, .coords.x = 1.2, .coords.y = 2.2};

    payload_ptr = &payload;

    ret = lora_send(sx1280_dev, payload_ptr, sizeof(payload));
    if(ret < 0)
    {
        LOG_ERR("LoRa send failed");
        return false;
    }
    return true;
}


void receive(const struct device *sx1280_dev, uint8_t *payload_ptr)
{
    int16_t rssi;
	int8_t snr;
    int len = -1;
    while(len < 0)
    {
        len = lora_recv(sx1280_dev, payload_ptr, MAX_DATA_LEN, K_MSEC(500),
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
    //Getting Device Id
    uint8_t hwid[4];
    uint32_t host_id;
    ssize_t length = hwinfo_get_device_id(hwid, sizeof(hwid));
    host_id = (hwid[0] << 24) | (hwid[1] << 16) | (hwid[2] << 8) | hwid[3];

    const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    struct lora_modem_config config;

    // Payload declaration
    struct Payload payload = {  .host_id = host_id, 
                                .operation = RANGING_INIT,
                                .coords.flag = false, .coords.x = -1, .coords.y = -1};;
    uint8_t *payload_ptr;
    payload_ptr = &payload;

    struct Anchor anchor_node;
    struct lora_ranging_params ranging_result;
    
    // General Variables
    int16_t rssi;
	int8_t snr;
    int ret, len;
    int count;


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
    config.tx = true;

    ret = lora_config(lora_dev, &config);
	if (ret < 0) {
		LOG_ERR("LoRa config failed");
		return;
	}
    begin:
	while (1) {
        // Make a broadcast to initiate ranging
        while(!(broadcast_ranging_req(lora_dev)));

        // Receive Mode
        do {
            len = lora_recv(lora_dev, payload_ptr, MAX_DATA_LEN, K_MSEC(1000),
				&rssi, &snr);
            if (len > 0)
            {
                // Add Anchor to Queue if payload has RANGING_ACK
                if(payload.operation == RANGING_ACK)
                {
                    anchor_node.host_id = payload.host_id;
                    anchor_node.coords = payload.coords;
                    anchor_node.next = NULL;
                    add_anchor(anchor_node);
                }
                
            }
        }
        while(!(len < 0));

        // Received all Anchors
        // Check if we have sufficient Anchors replied (** Minimum 3 **)
        count = get_anchor_count();
        if (!(count >= 3))
        {
            LOG_ERR("Not Sufficient(only %d) Anchors to proceed further.", count);
            goto begin;
        }

        // Start Ranging
        ret = lora_setup_ranging(lora_dev, &config, host_id, ROLE_SENDER);
        if(ret != true) {
            LOG_ERR("LoRa config failed.");
            return;
        }

        
        ranging_result = lora_transmit_ranging(lora_dev, &config, RangingAddress, TxtimeoutmS);

        
        // 
        k_sleep(K_MSEC(1000));
	}
}