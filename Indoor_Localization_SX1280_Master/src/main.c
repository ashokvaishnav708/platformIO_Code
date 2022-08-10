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
/* Hardware Addresses of currently available nrf52840 Anchors 
raspi03     48 d7 41 a7, 
raspi06     1c 1b 65 6e, 
raspi07     1d 5a 2a 62, 
raspi10     e7 32 e3 a5, 
raspi12     66 18 2c 94, 
raspi16     50 9a 12 7c,
*/

#define MAX_ANCHORS 06

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

#define START_RANGING           0x07
#define ALIVE                   0x08
#define ALIVE_ACK               0x09
#define ALL_DONE_PKT            0x06
#define ANCHOR_PKT              0x10
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

int get_alive_anchors_count()
{
    int count = 0;
    int alive = 0;
    while(count < MAX_ANCHORS)
    {
        if (anchor_alive[count] == true)
        {
            alive ++;
            LOG_INF(" Anchor : %x", anchor_id[count]);
        }
        count++;
    }
    return alive;
}

void reset_anchors_status()
{
    int count = 0;
    while(count < MAX_ANCHORS)
    {
        anchor_alive[count] = false;
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

    //struct Anchor anchor[MAX_ANCHORS];
    

    const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    struct lora_modem_config config;

    int ret, len;
    int16_t rssi;
	int8_t snr;
    bool ranging_req_possible = false;
    int count = 0;
    uint32_t ranging_req_id = 0x0000;
    uint8_t operation = RECEIVE;

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

    count = 0;
    /*
    while(count < MAX_ANCHORS)
    {
        anchor_alive[count] = true;
        count++;
    }
    count = 0;
    */
    //BEGIN:
    while(1)
    {
        
        switch (operation)
        {
            case RECEIVE:   LOG_INF("RECEIVE MODE.");
                            len = lora_recv(lora_dev, payload_ptr, MAX_DATA_LEN, K_MSEC(5000),
                                    &rssi, &snr);
                            //LOG_INF("LEN : %d", len);
                            if(len < 0)
                            {
                                if (len == -(EAGAIN)) {
                                    operation = ALIVE;
                                    ranging_req_possible = false;
                                    reset_anchors_status();
                                    count = 0;
                                } 
                                else operation = RECEIVE;
                                break;
                            }
                            operation = payload.operation;
                            break;
            
            case ALIVE: if (count == MAX_ANCHORS) {
                            if (get_alive_anchors_count() == 0) ranging_req_possible = false;
                            else ranging_req_possible = true;
                            operation = RECEIVE;
                            count = 0;
                            break;
                        }
                        payload.operation = ALIVE;
                        payload.host_id = anchor_id[count];
                        get_host_coordinates(payload.host_id, &payload.coords);

                        anchor_alive[count] = false;
                        k_sleep(K_MSEC(20));
                        ret = lora_send(lora_dev, payload_ptr, sizeof(payload));
                        k_sleep(K_MSEC(15));

                        LOG_INF("ALIVE PKT SENT.");
                        operation = ALIVE_ACK;
                        break;

            case ALIVE_ACK: len = lora_recv(lora_dev, payload_ptr, MAX_DATA_LEN, K_MSEC(50),
                                    &rssi, &snr);
                            if(len < 0)
                            {
                                LOG_INF("NOT ALIVE.");
                                operation = ALIVE;
                                
                            }
                            else if(payload.operation == ALIVE_ACK)
                            {
                                if(payload.host_id == anchor_id[count]) {
                                    anchor_alive[count] = true;
                                    LOG_INF("ANCHOR ALIVE ACK.");
                                    //count++;
                                    operation = ALIVE;
                                }        
                            }
                            count++;
                            break;
                            
            case RANGING_INIT:  LOG_INF("RANGING REQUEST RECEIVED.");
                                if (ranging_req_possible)
                                {
                                    LOG_INF("RANGING POSSIBLE");
                                    ranging_req_id = payload.host_id;
                                    count = 0;
                                    operation = ANCHOR_PKT;
                                    ranging_req_possible = false;
                                }

                                //else if (count == 0) operation = ALIVE;
                                else operation = ALIVE;
                                break;
                                
            case ANCHOR_PKT:    if (count == MAX_ANCHORS)
                                {
                                    payload.operation = ALL_DONE_PKT;
                                    payload.host_id = ranging_req_id;
                                    payload.coords = dev_coords;

                                    k_sleep(K_MSEC(20));
                                    ret = lora_send(lora_dev, payload_ptr, sizeof(payload));
                                    k_sleep(K_MSEC(15));

                                    count = 0;
                                    operation = START_RANGING;
                                    //ranging_req_possible =  true;
                                    break;
                                }
                                
                                else if (anchor_alive[count] == true)
                                {
                                    payload.host_id = anchor_id[count];
                                    payload.operation = ANCHOR_PKT;
                                    get_host_coordinates(anchor_id[count], &payload.coords);

                                    k_sleep(K_MSEC(20));
                                    ret = lora_send(lora_dev, payload_ptr, sizeof(payload));
                                    k_sleep(K_MSEC(15));
                                    operation = ANCHOR_PKT;
                                    
                                }
                                count++;
                                break;
            
            case START_RANGING: payload.operation = START_RANGING;
                                payload.host_id = ranging_req_id;
                                payload.coords = dev_coords;
                                
                                k_sleep(K_MSEC(20));
                                ret = lora_send(lora_dev, payload_ptr, sizeof(payload));
                                k_sleep(K_MSEC(15));

                                ranging_req_possible =  false;
                                reset_anchors_status();
                                count = 0;
                                operation = RECEIVE;
                                break;
            /*
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
