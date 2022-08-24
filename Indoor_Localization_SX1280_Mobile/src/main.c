

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

#define ROLE_SENDER 0x01

#define MAX_DATA_LEN 249

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL

//Operations
#define RANGING_INIT            0x00
#define RANGING_DONE            0x01
#define RANGING_ACK             0x02
#define RANGIGN_MODE_MASTER     0x03
#define RECEIVE                 0x05

#define ALL_DONE_PKT            0x06
#define START_RANGING           0x07
#define ANCHOR_PKT              0x10
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
    float distance;
    struct Anchor *next;
};

/* ********* Queue Operations ********** */

struct Anchor *front = NULL;
struct Anchor *rear = NULL;
int anchor_count = 0;

bool already_existing(uint32_t host_id)
{
    struct Anchor *temp_anchor;
    temp_anchor = front;
    do {
        if(temp_anchor->host_id == host_id)
        {
            return true;
        }
        temp_anchor = temp_anchor->next;
    }while(temp_anchor != NULL);
    return false;
}

bool add_anchor(uint32_t host_id, struct Coordinates coords)
{
    struct Anchor *n_anchor = malloc(sizeof(struct Anchor));
    n_anchor->coords = coords;
    n_anchor->host_id = host_id;
    n_anchor->distance = -1;
    n_anchor->next = NULL;

    if(rear == NULL)
    {
        front = n_anchor;
        rear = n_anchor;
    }
    else {
        if(already_existing(n_anchor->host_id)) return false;
        rear->next = n_anchor;
        rear = rear->next;
    }
    anchor_count++;
    return true;
}

void remove_anchor(struct Anchor *prev_anchor, struct Anchor *anchor_ptr)
{
    if (anchor_ptr->next == NULL) {
        if (prev_anchor == NULL) rear = NULL;
        else rear = prev_anchor;
    }
    else {
        prev_anchor->next = anchor_ptr->next;
    }
    free(anchor_ptr);
    anchor_count--;
}

void remove_all_anchors()
{
    struct Anchor *temp_anchor;
    if (front != NULL)
    {
        do {
            temp_anchor = front;
            front = front->next;
            free(temp_anchor);
        }while(front != NULL);
    }
    
    rear = NULL;
    anchor_count = 0;
}


void show_anchors()
{
    struct Anchor *temp_anchor;
    if(front != NULL)
    {
        temp_anchor = front;
        do {
            LOG_INF("Anchor ID: %x  Distance: %d", temp_anchor->host_id, temp_anchor->distance);
            temp_anchor = temp_anchor->next;
        }while(temp_anchor != NULL);
    }
    else
    {
        LOG_INF("No Anchors Yet.");
    }
}


/*
bool broadcast_ranging_req(const struct device *sx1280_dev, uint32_t host_id, struct Coordinates coords)
{
    int ret;
    uint8_t *payload_ptr;
   
    uint8_t data[MAX_DATA_LEN] = {RANGING_INIT, 
                                ((device_address >> 24u ) & 0xFFu),
                                ((device_address >> 16u ) & 0xFFu),
                                ((device_address >> 8u ) & 0xFFu),
                                (device_address & 0xFFu), };
    */
/*
    
    struct Payload payload = {  .host_id = host_id, 
                                .operation = RANGING_INIT,
                                .coords = coords};

    payload_ptr = &payload;

    ret = lora_send(sx1280_dev, payload_ptr, sizeof(payload));
    if(ret < 0)
    {
        LOG_ERR("LoRa send failed");
        return false;
    }
    return true;
}

*/
/*
void receive(const struct device *sx1280_dev, uint8_t *payload_ptr)
{
    int16_t rssi;
	int8_t snr;
    int len = -1;
    while(len < 0)
    {
        len = lora_recv(sx1280_dev, payload_ptr, MAX_DATA_LEN, K_MSEC(500),
				&rssi, &snr);
        if (!(len < 0))
        {
            LOG_INF("Received data: %x %x , (RSSI:%ddBm, SNR:%ddBm)",
                payload.dev_id, payload.operation, rssi, snr);
        }
    }
}
*/



float square(float x)
{
    return x*x;
}

bool equate_coordinates(struct Coordinates *coord1, struct Coordinates *coord2)
{
    if ((coord1->x) == (coord2->x) && (coord1->y) == (coord2->y)) return true;
    else return false;
}

bool coordinates_set_intersection(struct Coordinates *coord0, struct Coordinates *coord, struct Coordinates *coord_prime)
{
    if (equate_coordinates(coord0, coord)) return true;
    else if(equate_coordinates(coord0, coord_prime)) return true;
    else return false;    
}


void circles_intersection(struct Anchor *anchor1, struct Anchor *anchor2, struct Coordinates *coord, struct Coordinates *coord_prime)
{
    //struct Coordinates coord, coord_prime;
    float x2, y2, dx, dy;

    float dist, a, h;
    
    dx = anchor1->coords.x - anchor2->coords.x;
    dy = anchor1->coords.y - anchor2->coords.y;

    dist = hypot(dx, dy);    

    // Check if two circles are not same || circles do not meet || one circle is inside another one
    if ((dist == 0.0) || (dist > (anchor1->distance + anchor2->distance)) 
                        || (dist < fabs(anchor1->distance - anchor2->distance)))
    {
        coord->flag = false;
        coord_prime->flag = false;
    }
    else 
    {
        a = ((square(anchor1->distance) - square(anchor2->distance) + square(dist)) / (2.0 * dist));
        h = sqrt(square(anchor1->distance) - square(a));

        x2 = anchor1->coords.x + (dx * a/dist);
        y2 = anchor1->coords.y + (dy * a/dist);

        coord->x = x2 - (dy * h/dist);
        coord_prime->x = x2 + (dy * h/dist);
        coord->flag = true;
        coord->y = y2 + (dx * h/dist);
        coord_prime->y = y2 - (dx * h/dist);
        coord->flag = true;
    }
}

struct Coordinates calc_dev_location()
{
    struct Coordinates coord, coord_prime;
    struct Coordinates dev_coord, temp_coord;
    struct Anchor *anchor_ptr;
    bool first_iter = true;
    dev_coord.flag = NULL;
    temp_coord.flag = NULL;

    coord.flag = false;
    coord_prime.flag = false;

    anchor_ptr = front;
    circles_intersection(anchor_ptr, anchor_ptr->next, &dev_coord, &temp_coord);
    anchor_ptr = anchor_ptr->next;
    do {
        circles_intersection(anchor_ptr, anchor_ptr->next, &coord, &coord_prime);
        if (first_iter) {
            first_iter = false;
            if(coordinates_set_intersection(&temp_coord, &coord, &coord_prime)) {
                first_iter = true;
                dev_coord = temp_coord;
            }
        }
        if(!first_iter) {
            if ( !(coordinates_set_intersection(&dev_coord, &coord, &coord_prime)) ) {
                LOG_ERR("No intersection point.");
                dev_coord.flag = false;
                break;
            }
        }
        first_iter = false;
        anchor_ptr = anchor_ptr->next;
    }while(anchor_ptr->next != NULL);
    return dev_coord;
}


void main(void)
{
    //Getting Device Id
    uint8_t hwid[4];
    ssize_t length = hwinfo_get_device_id(hwid, sizeof(hwid));
    uint32_t host_id = (hwid[0] << 24) | (hwid[1] << 16) | (hwid[2] << 8) | hwid[3];

    struct Coordinates dev_coords = {.flag = false, .x = -1, .y = -1};

    const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    struct lora_modem_config config;

    // Payload declaration
    struct Payload payload;
    uint8_t *payload_ptr;
    payload_ptr = &payload;

    struct Anchor *anchor_ptr;
    struct Anchor *prev_anchor = NULL;
    struct Anchor *temp_anchor;
    struct lora_ranging_params ranging_result;
    
    // General Variables
    int16_t rssi;
	int8_t snr;
    int ret, len;
    bool ranging_done = false;
    uint8_t operation = RECEIVE;

    if (!device_is_ready(lora_dev)) {
		LOG_ERR("%s Device not ready", lora_dev->name);
		return;
	}

    config.frequency = 2445000000;
	config.bandwidth = BW_1600;
	config.datarate = SF_7;
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

    //BEGIN:
	while (1) {
        
        // Setup LoRa Device
        if(ranging_done)
        {
            ret = lora_config(lora_dev, &config);
            if (ret < 0) {
                LOG_ERR("LoRa config failed");
                return;
            }
            ranging_done = false;
        }
        
        switch(operation)
        {
            case RECEIVE:   LOG_INF("RECEIVE MODE.");
                            len = lora_recv(lora_dev, payload_ptr, MAX_DATA_LEN, K_MSEC(1000),
                                    &rssi, &snr);
                            if(len < 0)
                            {
                                if(len == -(EAGAIN)) {
                                    remove_all_anchors();
                                    operation = RANGING_INIT;
                                }
                                else operation = RECEIVE;
                            }
                            else
                            {
                                if (payload.operation == RANGING_INIT) operation = RECEIVE;
                                else operation = payload.operation;
                            }
                            break;
            case RANGING_INIT:  LOG_INF("RANGING INIT PKT BROADCASTED.");
                                payload.operation = RANGING_INIT;
                                payload.host_id = host_id;
                                payload.coords = dev_coords;
                                
                                k_sleep(K_MSEC(20));
                                ret = lora_send(lora_dev, payload_ptr, sizeof(payload));
                                k_sleep(K_MSEC(15));

                                operation = RECEIVE;
                                break;
            case ANCHOR_PKT:   if(!add_anchor(payload.host_id, payload.coords))
                                {
                                    LOG_INF("Received Already Existing Anchor.");
                                }
                                operation = RECEIVE;
                                break;
            case ALL_DONE_PKT:  if(payload.host_id != host_id)
                                {
                                    operation = RECEIVE;
                                    break;
                                }
                                LOG_INF("ALL ANCHORS RECEIVED.");
                                //show_anchors();
                                operation = RECEIVE;

                                break;
            case START_RANGING: if (payload.host_id != host_id)
                                {
                                    operation = RECEIVE;
                                    break;
                                }
                                k_sleep(K_MSEC(10));

                                lora_setup_ranging(lora_dev, &config, host_id, ROLE_SENDER);

                                anchor_ptr = front;
                                do {

                                    k_sleep(K_MSEC(10));
                                    ranging_result = lora_transmit_ranging(lora_dev, &config, (anchor_ptr->host_id));
                                    if(ranging_result.status != false)
                                    {
                                        anchor_ptr->distance = ranging_result.distance; // Distance in cm.
                                        
                                        prev_anchor = anchor_ptr;
                                        anchor_ptr = anchor_ptr->next;    
                                    }
                                    else {
                                        LOG_INF("RANGING FAILED.");
                                        anchor_ptr->distance = -1;

                                        temp_anchor = anchor_ptr;
                                        anchor_ptr = anchor_ptr->next;
                                        remove_anchor(prev_anchor, temp_anchor);
                                    }
                                    //LOG_INF(" Ranging Anchor : %x", anchor_ptr->host_id);
                                    
                                }while(anchor_ptr != NULL);
                                show_anchors();
                                prev_anchor = NULL;
                                ranging_done = true;
                                /*
                                if(anchor_count >= 3)
                                {
                                    dev_coords = calc_dev_location();
                                    if (dev_coords.flag == false) LOG_ERR("Location Ambiguious");
                                    else LOG_INF("Device Location : (%d, %d)", dev_coords.x, dev_coords.y);
                                }
                                */
                                operation = RECEIVE;
                                break;

            default: operation = RECEIVE;

        }
        /*
        broadcast:
        // Make a broadcast to initiate ranging
        payload.operation = RANGING_INIT;
        payload.host_id = host_id;
        payload.coords = dev_coords;
        do {
            ret = lora_send(lora_dev, payload_ptr, sizeof(payload));
        }while(ret < 0);
        //while(!(broadcast_ranging_req(lora_dev, host_id, dev_coords)));

        k_sleep(K_MSEC(15));
        RECV_MODE:
        // Receive Mode
        do {
            len = lora_recv(lora_dev, payload_ptr, MAX_DATA_LEN, K_MSEC(1000),
				&rssi, &snr);
            if (len > 0)
            {
                // Add Anchor to Queue if payload has RANGING_ACK
                if(payload.operation == RANGING_ACK)
                {
                    if(!(add_anchor(payload.host_id, payload.coords)))
                    {
                        LOG_INF("Received Already Existing Anchor.");
                    }
                }
                else
                {
                    len = -1;
                }
                
            }
        }
        while(!(len < 0));

        // Received all Anchors
        // Check if we have sufficient Anchors replied (** Minimum 3 **)
        //count = get_anchor_count();
        if (!(anchor_count >= 3))
        {
            LOG_ERR("Not Sufficient(only %d) Anchors to proceed further.", anchor_count);
            show_anchors();
            goto RECV_MODE;
        }

        LOG_INF("Modules Replied : %d", anchor_count);
        show_anchors();
        
    
        // Start Ranging
        ret = lora_setup_ranging(lora_dev, &config, host_id, ROLE_SENDER);
        if(ret != true) {
            LOG_ERR("LoRa config failed.");
            return;
        }

        anchor_ptr = front;
        do {
            ranging_result = lora_transmit_ranging(lora_dev, &config, anchor_ptr->host_id, TxtimeoutmS);
            if(ranging_result.status != false)
            {
                anchor_ptr->distance = ranging_result.distance; // Distance in cm.
                
            }
            else{
                anchor_ptr->distance = 0.0;
            }
            anchor_ptr = anchor_ptr->next;

        }while(anchor_ptr != NULL);


    

        // Circle Algorithm

        //dev_coords = calc_device_location();

        k_sleep(K_MSEC(1000));
        */
	}
}