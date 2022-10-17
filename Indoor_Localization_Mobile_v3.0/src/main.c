

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

bool equate_coordinates(struct Coordinates coord1, struct Coordinates coord2)
{
    if ((coord1.x) == (coord2.x) && (coord1.y) == (coord2.y))
        return true;
    else
        return false;
}

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

/*****************    Intersection Points Queue Operation     *****************/

struct IPs
{
    struct Coordinates coords;
    struct IPs *next;
} ips_list;

void add_intersection(struct IPs *ips_front, struct IPs *ips_rear, struct Coordinates coords)
{
    struct IPs *n_ip = malloc(sizeof(struct IPs));
    n_ip->coords = coords;
    n_ip->next = NULL;
    if (ips_rear == NULL)
    {
        ips_front = n_ip;
        ips_rear = n_ip;
    }
    else
    {
        ips_rear->next = n_ip;
        ips_rear = ips_rear->next;
    }
}

void remove_all_intersections(struct IPs *ips_front, struct IPs *ips_rear)
{
    struct IPs *temp_ip;
    if (ips_front != NULL)
    {
        do
        {
            temp_ip = ips_front;
            ips_front = ips_front->next;
            free(temp_ip);
        } while (ips_front != NULL);
    }
    ips_rear = NULL;
}

bool equate_intersections(struct IPs *ips_front, struct IPs *ips_rear, struct Coordinates coords)
{
    struct IPs *temp_ip;
    temp_ip = ips_front;
    do
    {
        if (equate_coordinates(temp_ip->coords, coords))
        {
            return true;
        }
        temp_ip = temp_ip->next;
    } while (temp_ip != NULL);
    return false;
}

void remove_duplicate_intersections(struct IPs *ips_front, struct IPs *ips_rear, struct IPs *new_ips_front, struct IPs *new_ips_rear)
{
    struct IPs *temp_ip;
    if (ips_front != NULL)
    {
        do
        {
            temp_ip = ips_front;
            if (new_ips_rear == NULL)
            {
                add_intersection(new_ips_front, new_ips_rear, temp_ip->coords);
            }
            else
            {
                if (!(equate_intersections(new_ips_front, new_ips_rear, temp_ip->coords)))
                {
                    add_intersection(new_ips_front, new_ips_rear, temp_ip->coords);
                }
            }
            ips_front = ips_front->next;
            free(temp_ip);
        } while (ips_front != NULL);
    }
    ips_rear = NULL;
}

/****************************************************/

// Circle Intersection Code
float square(float x)
{
    return x * x;
}

/*
bool coordinates_set_intersection(struct Coordinates *coord0, struct Coordinates *coord, struct Coordinates *coord_prime)
{
    if (equate_coordinates(coord0, coord))
        return true;
    else if (equate_coordinates(coord0, coord_prime))
        return true;
    else
        return false;
}
*/

void get_centroid(struct IPs *ips_front, struct IPs *ips_rear, struct Coordinates *coords)
{
    struct IPs *temp_ip;
    int count = 0;
    temp_ip = ips_front;
    do
    {
        coords->x += temp_ip->coords.x;
        coords->y += temp_ip->coords.y;

        count++;
        temp_ip = temp_ip->next;
    } while (temp_ip != NULL);

    coords->x = (coords->x / count);
    coords->y = (coords->y / count);
    coords->flag = true;
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
    if ((dist == 0.0 && anchor1->distance == anchor2->distance) || (dist > (anchor1->distance + anchor2->distance)) || (dist < fabs(anchor1->distance - anchor2->distance)))
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

        coord->x = ceil(x2 - (dy * h / dist));
        coord_prime->x = ceil(x2 + (dy * h / dist));
        coord->flag = true;

        coord->y = ceil(y2 + (dx * h / dist));
        coord_prime->y = ceil(y2 - (dx * h / dist));
        coord->flag = true;
    }
}

bool is_inside(struct Anchor *anchor, struct Coordinates coords)
{
    float circle_x = anchor->coords.x;
    float circle_y = anchor->coords.y;

    float x = coords.x;
    float y = coords.y;

    if (square(x - circle_x) + square(y - circle_y) <= square((anchor->distance)) + 1)
        return true;
    else
        return false;
}

int is_inside_circles(struct Coordinates coords)
{
    int count = 0;
    struct Anchor *temp_anchor;
    temp_anchor = front;
    do
    {
        if (temp_anchor->distance > 0)
        {
            if (is_inside(temp_anchor, coords))
            {
                count++;
            }
        }
        temp_anchor = temp_anchor->next;
    } while (temp_anchor != NULL);
    return false;
}

void get_polygon(struct IPs *ips_front, struct IPs *ips_rear, struct IPs *new_ips_front, struct IPs *new_ips_rear, int error, int active_anchors)
{
    struct IPs *temp_ip;
    do
    {
        temp_ip = ips_front;
        if (is_inside_circles(temp_ip->coords) >= (active_anchors - error))
        {
            add_intersection(new_ips_front, new_ips_rear, temp_ip->coords);
        }
        ips_front = temp_ip->next;
        free(temp_ip);
    } while (ips_front != NULL);
}

void calc_dev_location(struct Coordinates *dev_coord)
{
    struct Coordinates coord, coord_prime;
    struct Anchor *anchor_ptr_i;
    struct Anchor *anchor_ptr_j;
    // bool first_iter = true;
    // dev_coord.flag = NULL;
    // temp_coord.flag = NULL;

    struct IPs *ips_front = NULL;
    struct IPs *ips_rear = NULL;

    struct IPs *ips_filtered_front = NULL;
    struct IPs *ips_filtered_rear = NULL;

    struct IPs *ips_polygon_front = NULL;
    struct IPs *ips_polygon_rear = NULL;

    coord.flag = false;
    coord_prime.flag = false;

    int error_rate = 0;
    int active_anchors = 0;

    anchor_ptr_i = front;
    do
    {
        if (anchor_ptr_i->distance > 0)
        {
            anchor_ptr_j = front;
            do
            {
                if (anchor_ptr_i != anchor_ptr_j)
                {
                    if (anchor_ptr_j->distance > 0)
                    {
                        circles_intersection(anchor_ptr_i, anchor_ptr_j, &coord, &coord_prime);
                        if (!(coord.flag) && !(coord_prime.flag))
                        {
                            if (ips_front == NULL)
                            {
                                add_intersection(ips_front, ips_rear, coord);
                                add_intersection(ips_front, ips_rear, coord_prime);
                            }
                        }
                    }
                }

                anchor_ptr_j = anchor_ptr_j->next;
            } while (anchor_ptr_j != NULL);

            active_anchors++;
        }

        anchor_ptr_i = anchor_ptr_i->next;
    } while (anchor_ptr_i != NULL);

    // Below function dumps the non-duplicate data in new IPs queue and frees the previous IPs list
    remove_duplicate_intersections(ips_front, ips_rear, ips_filtered_front, ips_filtered_rear);

    get_polygon(ips_filtered_front, ips_filtered_rear, ips_polygon_front, ips_polygon_rear, error_rate, active_anchors);

    get_centroid(ips_polygon_front, ips_polygon_rear, dev_coord);

    remove_all_intersections(ips_polygon_front, ips_polygon_rear);
}

// main function
void main(void)
{
    // Getting Device Id
    uint8_t hwid[4];
    ssize_t length;
    length = hwinfo_get_device_id(hwid, sizeof(hwid));
    uint32_t host_id = (hwid[0] << 24) | (hwid[1] << 16) | (hwid[2] << 8) | hwid[3];

    struct Coordinates dev_coords = {.flag = false, .x = -1, .y = -1};

    const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
    struct lora_modem_config config;

    // Payload declaration
    struct Payload payload;
    uint8_t *payload_ptr;
    payload_ptr = &payload;

    struct Anchor *anchor_ptr;
    // struct Anchor *prev_anchor = NULL;
    // struct Anchor *temp_anchor;
    struct lora_ranging_params ranging_result;

    // General Variables
    int16_t rssi;
    int8_t snr;
    int ret, len;
    bool ranging_done = false;
    uint8_t operation = RECEIVE;

    float sum = 0;
    float avg_fact = 0;
    int samples = 0;
    float ratio = 2570 / 1992;

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
            if (anchor_count > 2)
                operation = START_RANGING;
            else
                operation = RECEIVE;

            break;

        case START_RANGING:
            // k_sleep(K_MSEC(10));

            anchor_ptr = front;
            do
            {
                if (!ranging_done)
                    lora_setup_ranging(lora_dev, &config, host_id, ROLE_SENDER);
                // k_sleep(K_MSEC(30));
                samples = 0;
                sum = 0;
                avg_fact = 0;

                while (samples < 10)
                {
                    ranging_result = lora_transmit_ranging(lora_dev, &config, (anchor_ptr->host_id));
                    if (ranging_result.status != false)
                    {
                        sum = sum + ranging_result.distance;
                        avg_fact++;
                    }

                    samples++;
                }
                if (sum > 0.0)
                {
                    anchor_ptr->distance = ceil((sum / avg_fact) * ratio * 0.25); // Distance in pixels via ratio multiplication.
                    anchor_ptr->RSSI = ranging_result.RSSIVal;
                }
                else
                {
                    anchor_ptr->distance = -1;
                    anchor_ptr->RSSI = 0;
                }

                anchor_ptr = anchor_ptr->next;

            } while (anchor_ptr != NULL);

            show_anchors();
            // prev_anchor = NULL;
            calc_dev_location(&dev_coords);
            ranging_done = true;
            LOG_INF("Device Location :(%d, %d).", dev_coords.x, dev_coords.y);
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