#include <device.h>
#include <drivers/lora.h>
#include <errno.h>
#include <sys/util.h>
#include <zephyr.h>
#include <logging/log.h>
#include <drivers/hwinfo.h>
#include <stdlib.h>
#include <math.h>

#define DEFAULT_RADIO_NODE DT_ALIAS(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
	     "No default LoRa radio specified in DT");

#define MAX_DATA_LEN 255

//Anchors
#define MAX_ANCHORS 04

#define RASPI03     0x48d741a7
#define RASPI06     0x1c1b656e
#define RASPI07     0x1d5a2a62
#define RASPI10     0xe732e3a5
#define RASPI12     0x66182c94
#define RASPI16     0x509a127c
#define MASTER      0x33d6f416
//


//Operations
#define RANGING_INIT            0x00
#define RANGING_DONE            0x01
#define RANGING_ACK             0x02
#define RANGIGN_MODE_MASTER     0x03
#define SEND_ACK_PKT            0x04
#define RECEIVE                 0x05

#define ALIVE                   0x08
#define ALIVE_ACK               0x09
#define ALL_DONE_PKT            0x06
#define NONE                    0xFF
//


LOG_MODULE_REGISTER(Indoor_Localization_Master);


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

const uint32_t anchor_id[MAX_ANCHORS] = {RASPI03, RASPI06, RASPI07, RASPI10, RASPI12, RASPI16};
bool anchor_alive[MAX_ANCHORS] = {0, 0, 0, 0, 0, 0};

struct Anchor {
    bool alive;
    uint32_t host_id;
    struct Coordinates coords;
};



void get_host_coordinates(uint32_t host_id, struct Coordinates *coords)
{
    switch (host_id)
    {
        case RASPI03:   coords->flag = true;
                        coords->x = 14.44;
                        coords->y = 19.92;
                        break;
        case RASPI06:   coords->flag = true;
                        coords->x = 19.93;
                        coords->y = 19.92;
                        break;
        case RASPI07:   coords->flag = true;
                        coords->x = 19.93;
                        coords->y = 12.71;
                        break;
        case RASPI10:   coords->flag = true;
                        coords->x = 19.93;
                        coords->y = 0.0;
                        break;
        case RASPI12:   coords->flag = true;
                        coords->x = 16.0;
                        coords->y = 13.0;
                        break;
        case RASPI16:   coords->flag = true;
                        coords->x = 0.0;
                        coords->y = 19.92;
                        break;
        default:    coords->flag = false;
                    coords->x = -1.0;
                    coords->y = -1.0;
    }
}

void show_alive_anchors()
{
    int count = 0;
    while(count < MAX_ANCHORS)
    {
        if (anchor_alive[count] == true)
        {
            LOG_INF(" Anchor : %x", anchor_id[count]);
        }
        count++;
    }
}

void main(void)
{

    // Hardware information gain
    uint8_t hwid[4];
    _ssize_t length = hwinfo_get_device_id(hwid, sizeof(hwid));
    uint32_t host_id = (hwid[0] << 24) | (hwid[1] << 16) | (hwid[2] << 8) | hwid[3];
    struct Coordinates dev_coords;

    struct Anchor anchor[MAX_ANCHORS];
    

    const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    struct lora_modem_config config;

    int ret, len;
    int16_t rssi;
	int8_t snr;
    int anchor_counter = 0;
    int count = 0;
    uint8_t operation = ALIVE;

    // Payload declaration
    struct Payload payload;
    uint8_t *payload_ptr;
    payload_ptr = &payload;

    get_host_coordinates(host_id, &dev_coords);

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

    // Setup LoRa Device
    ret = lora_config(lora_dev, &config);
    if (ret < 0) {
        LOG_ERR("LoRa config failed");
        return;
    }

    //
    /*
    while(anchor_counter < MAX_ANCHORS)
    {
        anchor[anchor_counter].alive = false;
        anchor[anchor_counter].host_id = anchor_id[anchor_counter]; 
        get_host_coordinates(anchor[anchor_counter].host_id, &anchor[anchor_counter].coords); 

        payload.operation = ALIVE;
        payload.host_id = anchor_id[anchor_counter];
        get_host_coordinates(payload.host_id, &payload.coords);

        k_sleep(K_MSEC(20));
        ret = lora_send(lora_dev, payload_ptr, sizeof(payload));
        if(ret < 0) {
            LOG_ERR("ALIVE PKT SENDING FAILED.");
        }
        k_sleep(K_MSEC(15));
        LOG_INF("ALIVE PKT SENT.");

        len = lora_recv(lora_dev, payload_ptr, MAX_DATA_LEN, K_MSEC(20),
                                    &rssi, &snr);
        if(len < 0)
        {
            anchor_counter ++;
            continue;
        }
        if (payload.operation == ALIVE_ACK && payload.host_id = anchor_id[anchor_counter])
        {
            anchor[anchor_counter].alive = true;
        }
        anchor_counter ++;
    }
*/
    count = 0;

    BEGIN:
    while(1)
    {
        
        switch (operation)
        {
            case RECEIVE:   LOG_INF("RECEIVE MODE.");
                            len = lora_recv(lora_dev, payload_ptr, MAX_DATA_LEN, K_MSEC(5000),
                                    &rssi, &snr);
                            if(len < 0)
                            {
                                if (len == EAGAIN) operation = ALIVE;
                                else operation = RECEIVE;
                                break;
                            }
                            operation = payload.operation;
                            break;
            
            case ALIVE: if (anchor_counter == MAX_ANCHORS) {
                            show_alive_anchors();
                            operation = RECEIVE;
                            count = 0;
                            break;
                        }
                        payload.operation = ALIVE;
                        payload.host_id = anchor_id[count];
                        get_host_coordinates(payload.host_id, &payload.coords);

                        k_sleep(K_MSEC(20));
                        ret = lora_send(lora_dev, payload_ptr, sizeof(payload));
                        if(ret < 0) {
                            k_sleep(K_MSEC(15));
                            operation = ALIVE;
                            LOG_ERR("ALIVE PKT SENDING FAILED.");
                            break;
                        }
                        k_sleep(K_MSEC(15));
                        LOG_INF("ALIVE PKT SENT.");
                        operation = ALIVE_ACK;
                        break;
            case ALIVE_ACK: len = lora_recv(lora_dev, payload_ptr, MAX_DATA_LEN, K_MSEC(20),
                                    &rssi, &snr);
                            if(len < 0)
                            {
                                LOG_INF("NOT ALIVE.");
                                count++;
                                operation = ALIVE;
                                break;
                            }
                            if(payload.operation == ALIVE_ACK)
                            {
                                if(payload.host_id == anchor_id[anchor_counter]) {
                                    anchor_alive[count] = true;
                                    LOG_INF("ANCHOR ALIVE ACK.");
                                    anchor_counter++;
                                    operation = ALIVE;
                                    break;
                                }        
                            }
                            count++;
                            break;
                            
            
            /*
            case RANGING_INIT:  operation = SEND_ACK_PKT;
                                //k_sleep(K_MSEC(20));
                                break;
            
            case SEND_ACK_PKT:  if(anchor_counter == MAX_ANCHORS)
                                {
                                    payload.operation = ALL_DONE_PKT;
                                    payload.host_id = host_id;
                                    payload.coords = dev_coords;

                                    k_sleep(K_MSEC(30));
                                    ret = lora_send(lora_dev, payload_ptr, sizeof(payload));
                                    if(ret < 0) {
                                        LOG_ERR("Ranging Ack failed.");
                                    }
                                    k_sleep(K_MSEC(20));
                                    LOG_INF("SENT ALL DONE PKT.");
                                    anchor_counter = 0;
                                    operation = RECEIVE;
                                    break;
                                }
                                payload.operation = SEND_ACK_PKT;
                                payload.host_id = anchor_id[anchor_counter];
                                get_host_coordinates(payload.host_id, &payload.coords);

                                k_sleep(K_MSEC(30));
                                ret = lora_send(lora_dev, payload_ptr, sizeof(payload));
                                if(ret < 0) {
                                    LOG_ERR("Ranging Ack failed.");
                                }
                                k_sleep(K_MSEC(20));
                                LOG_INF("SENT ACK PKT.");
                                operation = RECEIVE;
                                break;   
            case RANGING_ACK:   if(payload.host_id != anchor_id[anchor_counter])
                                {
                                    operation = RECEIVE;
                                    break;
                                }
                                anchor_counter++;
                                LOG_INF("RANGING ACK PKT RECEIVED.");
                                operation = SEND_ACK_PKT;
                                break;
            */
            default: operation = RECEIVE;
        }

    }
}
