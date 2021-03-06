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

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>

LOG_MODULE_REGISTER(sx1280_send);

char data[MAX_DATA_LEN] = {'h', 'e', 'l', 'l', 'o', 'w', 'o', 'r', 'l', 'd'};
const uint16_t TXtimeoutmS = 5000;
const uint16_t RXtimeoutmS = 0xFFFF;

void main(void)
{
	const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
	struct lora_modem_config config;
	int ret;
    uint32_t RangingAddress = 16;
    struct lora_ranging_params rangingResult;

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

	// printk("pre config\n");
    #ifdef SENDER
	ret = lora_setup_ranging(lora_dev, &config, RangingAddress, ROLE_SENDER);
    #endif

    #ifdef RECEIVER
    ret = lora_setup_ranging(lora_dev, &config, RangingAddress, ROLE_RECEIVER);
    #endif

	if (ret != true) {
		LOG_ERR("LoRa config failed");
		return;
	}

	while (1) {
        #ifdef SENDER
		// printk("sending...\n");
		rangingResult = lora_transmit_ranging(lora_dev, &config, RangingAddress, TXtimeoutmS);
        printk("\nDistance : %f", rangingResult.distance);
		if (rangingResult.status == false) {
			LOG_ERR("LoRa ranging failed");
			return;
		}
        LOG_INF("Distance : %f", rangingResult.distance);
        LOG_INF(", RSSIReg. : %x", rangingResult.RSSIReg);
        LOG_INF(", RSSI : %d dBm", rangingResult.RSSIVal);
		// printk("Data sent!\n");

		/* Send data at 1s interval */
        #endif
        #ifdef RECEIVER
        if ( lora_receive_ranging(lora_dev, &config, RangingAddress, RXtimeoutmS) )
        {
            LOG_INF("Response Sent");
        }
        #endif
		k_sleep(K_MSEC(1000));
	}
}
