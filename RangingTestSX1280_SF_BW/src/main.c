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



#define ROLE_SENDER 0x01
#define ROLE_RECEIVER 0x00



//#define SENDER
#define RECEIVER

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>

LOG_MODULE_REGISTER(sx1280_ranging);

struct Payload {
    enum lora_datarate datarate;
    enum lora_signal_bandwidth bandwidth;
    enum lora_coding_rate cr;
};

void main(void)
{

    uint32_t RangignAddress = 0x48d741a6;

    const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
	struct lora_modem_config config;

    #ifdef SENDER
    enum lora_datarate datarate[6] = {SF_5, SF_6, SF_7, SF_8, SF_9, SF_10};
    enum lora_signal_bandwidth bandwidth[3] = {BW_0400, BW_0800, BW_1600};
    enum lora_coding_rate cr[4] = {CR_4_5, CR_4_6, CR_4_7, CR_4_8};

	struct lora_ranging_params rangingResult;
    int i, j, k, l;
    #endif

	int ret;
    #ifdef RECEIVER
    int16_t rssi;
    int8_t snr;
    int i;
    #endif
    bool end_flag = false;

    bool ranging_done = true;

    

    struct Payload payload;
    uint8_t *payload_ptr;
    payload_ptr = &payload;

 



    if (!device_is_ready(lora_dev)) {
		LOG_ERR("%s Device not ready.", lora_dev->name);
		return;
	}

	config.frequency = 2445000000;
	config.bandwidth = BW_0400;
	config.datarate = SF_5;
	config.preamble_len = 12;
	config.coding_rate = CR_4_5;
	config.tx_power = 10;
    #ifdef SENDER
	config.tx = true;
    #endif
    #ifdef RECEIVER
    config.tx = false;
    #endif

    ret = lora_config(lora_dev, &config);
    if (ret < 0) {
        LOG_ERR("LoRa config failed");
        return;
    }

    #ifdef SENDER
    k_sleep(K_MSEC(10000));
    #endif
    
    while(!end_flag) 
    {
        #ifdef SENDER
        for (i=0; i<6;i++) 
        {
            for (j=0; j<3; j++)
            {
                for (k=0; k<4; k++)
                {
                    if (ranging_done)
                    {
                        ranging_done = false;
                        lora_config(lora_dev, &config);

                        payload.datarate = datarate[i];
                        payload.bandwidth = bandwidth[j];
                        payload.cr = cr[k];

                        config.datarate = datarate[i];
                        config.bandwidth = bandwidth[j];
                        config.coding_rate = cr[k];
                        
                        k_sleep(K_MSEC(10));
                        lora_send(lora_dev, payload_ptr, sizeof(payload));
                        k_sleep(K_MSEC(20));
                    }
                    
                    ret = lora_setup_ranging(lora_dev, &config, RangignAddress, ROLE_SENDER);

                    for (l=0; l<100; l++)
                    {
                        k_sleep(K_MSEC(10));
                        rangingResult = lora_transmit_ranging(lora_dev, &config, RangignAddress);
                        LOG_INF("SF: %d BW: %d CR: %d RSSI: %d Dist: %d",
                            datarate[i], bandwidth[j], cr[k], rangingResult.RSSIVal, rangingResult.distance);
                    }
                    ranging_done = true;
                }
            }
            if(i == 5) end_flag = true;
        }
        #endif
        #ifdef RECEIVER
        if(ranging_done)
        {
            ranging_done = false;
            lora_config(lora_dev, &config);

            lora_recv(lora_dev, payload_ptr, sizeof(payload), K_FOREVER, &rssi, &snr);

            config.bandwidth = payload.bandwidth;
            config.datarate = payload.datarate;
            config.coding_rate = payload.cr;
        }
        

        lora_setup_ranging(lora_dev, &config, RangignAddress, ROLE_RECEIVER);
        // add for loop
        for(i=0; i<100; i++)
        {
            lora_receive_ranging(lora_dev, &config, RangignAddress, K_FOREVER);
            ranging_done = true;
        }
        #endif
    }
}
