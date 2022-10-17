

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

// Operations
#define RANGING_INIT 0x00
#define RANGING_DONE 0x01
#define RANGING_ACK 0x02
#define RANGIGN_MODE_MASTER 0x03
#define RECEIVE 0x05

#define ALL_DONE_PKT 0x06
#define START_RANGING 0x07
#define ANCHOR_PKT 0x10
#define RE_RANGING_PKT 0x11
#define NONE 0xFF
//

LOG_MODULE_REGISTER(Indoor_Localization_Mobile);

const uint16_t TxtimeoutmS = 5000;
const uint16_t RxtimeoutmS = 0xFFFF;
const uint32_t device_address = 01;

struct __attribute__((__packed__)) Coordinates
{
    bool flag; //  Validated Coordinates if TRUE else not validated
    float x;
    float y;
};

/*
************ Payload Format ************
DEVICE_ID | OPERATION | DATA_POINTER
*/

struct __attribute__((__packed__)) Payload
{
    uint32_t host_id;
    uint8_t operation;
    struct Coordinates coords;
};

struct Anchor
{
    uint32_t host_id;
    struct Coordinates coords;
    float distance;
    // float re_distance;
    int16_t RSSI;
    // int16_t re_RSSI;
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
    do
    {
        if (temp_anchor->host_id == host_id)
        {
            return true;
        }
        temp_anchor = temp_anchor->next;
    } while (temp_anchor != NULL);
    return false;
}

bool add_anchor(uint32_t host_id, struct Coordinates coords)
{
    struct Anchor *n_anchor = malloc(sizeof(struct Anchor));
    n_anchor->coords = coords;
    n_anchor->host_id = host_id;
    n_anchor->distance = -1;
    // n_anchor->re_distance = -1;
    n_anchor->RSSI = 0;
    // n_anchor->re_RSSI = 0;
    n_anchor->next = NULL;

    if (rear == NULL)
    {
        front = n_anchor;
        rear = n_anchor;
    }
    else
    {
        if (already_existing(n_anchor->host_id))
            return false;
        rear->next = n_anchor;
        rear = rear->next;
    }
    anchor_count++;
    return true;
}

void remove_anchor(struct Anchor *prev_anchor, struct Anchor *anchor_ptr)
{
    if (prev_anchor == NULL)
    {
        front = anchor_ptr->next;
    }
    else
    {
        if (anchor_ptr->next == NULL)
        {
            if (prev_anchor == NULL)
                rear = NULL;
            else
                rear = prev_anchor;
        }
        else
        {
            prev_anchor->next = anchor_ptr->next;
        }
    }
    free(anchor_ptr);
    anchor_count--;
}

void remove_all_anchors()
{
    struct Anchor *temp_anchor;
    if (front != NULL)
    {
        do
        {
            temp_anchor = front;
            front = front->next;
            free(temp_anchor);
        } while (front != NULL);
    }

    rear = NULL;
    anchor_count = 0;
}

void show_anchors()
{
    struct Anchor *temp_anchor;
    if (front != NULL)
    {
        temp_anchor = front;
        do
        {
            LOG_INF("Anchor ID: %x RSSI: %d  Distance: %d",
                    temp_anchor->host_id, temp_anchor->RSSI, temp_anchor->distance);
            // LOG_INF("Re: Anchor ID: %x RSSI: %d  Distance: %d",
            //        temp_anchor->host_id, temp_anchor->re_RSSI, temp_anchor->re_distance);
            temp_anchor = temp_anchor->next;
        } while (temp_anchor != NULL);
    }
    else
    {
        LOG_INF("No Anchors Yet.");
    }
}

// Circle Intersection Code
float square(float x)
{
    return x * x;
}

bool equate_coordinates(struct Coordinates *coord1, struct Coordinates *coord2)
{
    if ((coord1->x) == (coord2->x) && (coord1->y) == (coord2->y))
        return true;
    else
        return false;
}

bool coordinates_set_intersection(struct Coordinates *coord0, struct Coordinates *coord, struct Coordinates *coord_prime)
{
    if (equate_coordinates(coord0, coord))
        return true;
    else if (equate_coordinates(coord0, coord_prime))
        return true;
    else
        return false;
}

void circles_intersection(struct Anchor *anchor1, struct Anchor *anchor2, struct Coordinates *coord, struct Coordinates *coord_prime)
{
    // struct Coordinates coord, coord_prime;
    float x2, y2, dx, dy;

    float dist, a, h;

    dx = anchor1->coords.x - anchor2->coords.x;
    dy = anchor1->coords.y - anchor2->coords.y;

    dist = hypot(dx, dy);

    // Check if two circles are not same || circles do not meet || one circle is inside another one
    if ((dist == 0.0) || (dist > (anchor1->distance + anchor2->distance)) || (dist < fabs(anchor1->distance - anchor2->distance)))
    {
        coord->flag = false;
        coord_prime->flag = false;
    }
    else
    {
        a = ((square(anchor1->distance) - square(anchor2->distance) + square(dist)) / (2.0 * dist));
        h = sqrt(square(anchor1->distance) - square(a));

        x2 = anchor1->coords.x + (dx * a / dist);
        y2 = anchor1->coords.y + (dy * a / dist);

        coord->x = x2 - (dy * h / dist);
        coord_prime->x = x2 + (dy * h / dist);
        coord->flag = true;
        coord->y = y2 + (dx * h / dist);
        coord_prime->y = y2 - (dx * h / dist);
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
    do
    {
        circles_intersection(anchor_ptr, anchor_ptr->next, &coord, &coord_prime);
        if (first_iter)
        {
            first_iter = false;
            if (coordinates_set_intersection(&temp_coord, &coord, &coord_prime))
            {
                first_iter = true;
                dev_coord = temp_coord;
            }
        }
        if (!first_iter)
        {
            if (!(coordinates_set_intersection(&dev_coord, &coord, &coord_prime)))
            {
                LOG_ERR("No intersection point.");
                dev_coord.flag = false;
                break;
            }
        }
        first_iter = false;
        anchor_ptr = anchor_ptr->next;
    } while (anchor_ptr->next != NULL);
    return dev_coord;
}

// main function
void main(void)
{
    // Getting Device Id
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

    if (!device_is_ready(lora_dev))
    {
        LOG_ERR("%s Device not ready", lora_dev->name);
        return;
    }

    config.frequency = 2445000000;
    config.bandwidth = BW_1600;
    config.datarate = SF_9;
    config.preamble_len = 12;
    config.coding_rate = CR_4_5;
    config.tx_power = 10;
    config.tx = true;

    // Setup LoRa Device
    ret = lora_config(lora_dev, &config);
    if (ret < 0)
    {
        LOG_ERR("LoRa config failed");
        return;
    }

    // BEGIN:
    while (1)
    {

        // Setup LoRa Device
        if (ranging_done)
        {
            ret = lora_config(lora_dev, &config);
            if (ret < 0)
            {
                LOG_ERR("LoRa config failed");
                return;
            }
            ranging_done = false;
        }

        switch (operation)
        {
        case RECEIVE:
            LOG_INF("RECEIVE MODE.");
            len = lora_recv(lora_dev, payload_ptr, MAX_DATA_LEN, K_MSEC(1000),
                            &rssi, &snr);
            if (len < 0)
            {
                if (len == -(EAGAIN))
                {
                    remove_all_anchors();
                    operation = RANGING_INIT;
                }
                else
                    operation = RECEIVE;
            }
            else
            {
                if (payload.operation == RANGING_INIT)
                    operation = RECEIVE;
                else
                    operation = payload.operation;
            }
            break;
        case RANGING_INIT:
            // remove_all_anchors();
            LOG_INF("RANGING INIT PKT BROADCASTED.");
            payload.operation = RANGING_INIT;
            payload.host_id = host_id;
            payload.coords = dev_coords;

            k_sleep(K_MSEC(20));
            ret = lora_send(lora_dev, payload_ptr, sizeof(payload));
            k_sleep(K_MSEC(30));

            operation = RECEIVE;
            break;
        case ANCHOR_PKT:
            if (!add_anchor(payload.host_id, payload.coords))
            {
                LOG_INF("Received Already Existing Anchor.");
            }
            operation = RECEIVE;
            break;
        case ALL_DONE_PKT:
            if (payload.host_id != host_id)
            {
                operation = RECEIVE;
                break;
            }
            LOG_INF("ALL ANCHORS RECEIVED.");
            // show_anchors();
            operation = START_RANGING;

            break;
        case START_RANGING:
            // k_sleep(K_MSEC(10));

            anchor_ptr = front;
            do
            {
                if (!ranging_done)
                    lora_setup_ranging(lora_dev, &config, host_id, ROLE_SENDER);
                // k_sleep(K_MSEC(30));
                ranging_result = lora_transmit_ranging(lora_dev, &config, (anchor_ptr->host_id));
                ranging_done = true;
                if (ranging_result.status != false)
                {
                    anchor_ptr->distance = ranging_result.distance; // Distance in cm.
                    anchor_ptr->RSSI = ranging_result.RSSIVal;

                    // lora_receive_ranging(lora_dev, &config, (anchor_ptr->host_id), K_MSEC(20));
                    //  k_sleep(K_MSEC(1));
                    // lora_config(lora_dev, &config);

                    // here, from payload coordinates we use x and y as distance and RSSI respectively.
                    /*
                    if (payload.coords.flag != false)
                    {
                        anchor_ptr->re_distance = payload.coords.x;
                        anchor_ptr->re_RSSI = (int16_t)payload.coords.y;
                    }
                    else
                    {
                        anchor_ptr->re_distance = -1;
                        anchor_ptr->re_RSSI = 0;
                    }
                    */
                    prev_anchor = anchor_ptr;
                    anchor_ptr = anchor_ptr->next;
                }
                else
                {
                    LOG_INF("RANGING FAILED.");
                    anchor_ptr->distance = -1;

                    // temp_anchor = anchor_ptr;
                    anchor_ptr = anchor_ptr->next;
                    // remove_anchor(prev_anchor, temp_anchor);
                }
                // LOG_INF(" Ranging Anchor : %x", anchor_ptr->host_id);

            } while (anchor_ptr != NULL);

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
            operation = START_RANGING;
            break;

        default:
            operation = RECEIVE;
        }
    }
}