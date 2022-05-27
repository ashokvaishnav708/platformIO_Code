/*
 * Copyright (c) 2019 Manivannan Sadhasivam
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/lora.h>
#include <errno.h>
#include <sys/util.h>
#include <zephyr.h>

#define DEFAULT_RADIO_NODE DT_ALIAS(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay),
	     "No default LoRa radio specified in DT");

#define MAX_DATA_LEN 10
#define ROLE_SENDER 0x01
#define ROLE_RECEIVER 0x00

#define SENDER
//#define RECEIVER

#ifdef SENDER
#define RANGING_SAMPLES 5
#endif

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>

LOG_MODULE_REGISTER(sx1280_ranging);

const uint16_t TxtimeoutmS = 5000;
const uint16_t RxtimeoutmS = 0xFFFF;


void main(void)
{
	const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
	struct lora_modem_config config;
	
	int ret;

	int samples;
	double total_dist;
    
	uint32_t RangingAddress = 16;
    
	#ifdef SENDER
	struct lora_ranging_params rangingResult;
    #endif

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
    #ifdef SENDER
	config.tx = true;
    #endif
    #ifdef RECEIVER
    config.tx = false;
    #endif

	// printk("pre config\n");
    #ifdef SENDER
	ret = lora_setup_ranging(lora_dev, &config, RangingAddress, ROLE_SENDER);
    #endif

    #ifdef RECEIVER
    ret = lora_setup_ranging(lora_dev, &config, RangingAddress, ROLE_RECEIVER);
    #endif

	if (ret != true) {
		LOG_ERR("LoRa config failed.");
		return;
	}

	while (1) {
        #ifdef SENDER
		total_dist = 0.0;
		samples = 0;

		while(samples < RANGING_SAMPLES)
		{
			rangingResult = lora_transmit_ranging(lora_dev, &config, RangingAddress, TxtimeoutmS);

			if (rangingResult.status == false) 
			{
				LOG_ERR("LoRa ranging failed.");
				goto sn;
			}

			LOG_INF("Distance : %d cm ", (int)rangingResult.distance);
			//LOG_INF(", RSSIReg. : %x", rangingResult.RSSIReg);
			//LOG_INF(", RSSI : %d dBm", rangingResult.RSSIVal);
			total_dist += rangingResult.distance;
			samples++;
		}
		sn:
		LOG_INF("Average Distance : %d cm ", (int)(total_dist/samples));

        #endif

        #ifdef RECEIVER
        if ( lora_receive_ranging(lora_dev, &config, RangingAddress, RxtimeoutmS) )
        {
            LOG_INF("Response Sent");
        }
        #endif
		k_sleep(K_MSEC(1000));
	}
}
