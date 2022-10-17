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

#define MAX_DATA_LEN 255

// operations

#define RECEIVE 0x00
#define CHNG_PARAMS 0x01
#define CHNG_PARAMS_ACK 0x02
#define START_RANGING 0x03
#define SEND_CHNG_PARAMS 0x04
#define RANGING_SLAVE_MODE 0x05

//#define SENDER
#define RECEIVER

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>

LOG_MODULE_REGISTER(sx1280_ranging);

struct Payload
{
	uint8_t operation;
	int dr_index;
	int bw_index;
	// enum lora_coding_rate cr;
};

void main(void)
{

	uint32_t RangignAddress = 0x48d741a6;

	const struct device *lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
	struct lora_modem_config config;
	enum lora_datarate datarate[6] = {SF_5, SF_6, SF_7, SF_8, SF_9, SF_10};
	enum lora_signal_bandwidth bandwidth[3] = {BW_0400, BW_0800, BW_1600};

#ifdef SENDER

	// enum lora_coding_rate cr[4] = {CR_4_5, CR_4_6, CR_4_7, CR_4_8};

	struct lora_ranging_params rangingResult;
	int bw_count = 0, SF_count = 0, test_count = 0;
#endif

	int ret;
	int16_t rssi;
	int8_t snr;
	int len;

	bool end_flag = false;
	uint8_t operation = RECEIVE;
	bool ranging_done = true;

	struct Payload payload;
	uint8_t *payload_ptr;
	payload_ptr = &payload;

	if (!device_is_ready(lora_dev))
	{
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
	if (ret < 0)
	{
		LOG_ERR("LoRa config failed");
		return;
	}

#ifdef SENDER
	k_sleep(K_MSEC(20000));
	operation = SEND_CHNG_PARAMS;
#endif

	while (!end_flag)
	{
		if (ranging_done)
		{
			ranging_done = false;
			lora_config(lora_dev, &config);
		}
#ifdef SENDER
		switch (operation)
		{
		case RECEIVE:
			LOG_INF("RECEIVE MODE.");
			len = lora_recv(lora_dev, payload_ptr, MAX_DATA_LEN, K_FOREVER,
							&rssi, &snr);
			if (len < 0)
			{
				operation = RECEIVE;
				break;
			}
			operation = payload.operation;
			break;

		case SEND_CHNG_PARAMS:
			LOG_INF("Sending CHNG_PARAMS");
			if (bw_count > 3)
			{
				end_flag = true;
				break;
			}
			payload.bw_index = bw_count;
			payload.dr_index = SF_count;
			payload.operation = CHNG_PARAMS;

			lora_send(lora_dev, payload_ptr, sizeof(payload));
			k_sleep(K_MSEC(20));

			LOG_INF("Sent CHNG_PARAMS.");

			config.bandwidth = bandwidth[bw_count];
			config.datarate = datarate[SF_count];
			lora_config(lora_dev, &config);

			LOG_INF("DEVICE PARAMS UPDATED.");
			operation = RECEIVE;
			break;

		case CHNG_PARAMS_ACK:
			LOG_INF("ACK_RECEIVED.");
			operation = START_RANGING;
			break;

		case START_RANGING:
			payload.operation = RANGING_SLAVE_MODE;
			lora_send(lora_dev, payload_ptr, sizeof(payload));
			k_sleep(K_MSEC(20));

			lora_setup_ranging(lora_dev, &config, RangignAddress, ROLE_SENDER);
			rangingResult = lora_transmit_ranging(lora_dev, &config, RangignAddress);
			if (rangingResult.status != false)
			{
				LOG_INF("SF: %d BW: %d RSSI: %d Dist: %d",
						datarate[SF_count], bandwidth[bw_count], rangingResult.RSSIVal, rangingResult.distance);
			}
			else
				LOG_ERR("Ranging Failed.");
			ranging_done = true;
			test_count += 1;
			if (test_count > 99)
			{
				test_count = 0;
				SF_count += 1;
				if (SF_count > 6)
				{
					SF_count = 0;
					bw_count += 1;
				}
				operation = SEND_CHNG_PARAMS;
			}
			else
				operation = START_RANGING;
			break;

		default:
			operation = RECEIVE;
		}
#endif

#ifdef RECEIVER
		switch (operation)
		{
		case RECEIVE:
			LOG_INF("RECEIVE MODE.");
			len = lora_recv(lora_dev, payload_ptr, MAX_DATA_LEN, K_FOREVER,
							&rssi, &snr);
			if (len < 0)
			{
				operation = RECEIVE;
				break;
			}
			operation = payload.operation;
			break;

		case CHNG_PARAMS:
			LOG_INF("Received Change Params.");
			config.bandwidth = bandwidth[payload.bw_index];
			config.datarate = datarate[payload.dr_index];
			// config.coding_rate = payload.cr;
			lora_config(lora_dev, &config);
			operation = CHNG_PARAMS_ACK;
			break;

		case CHNG_PARAMS_ACK:
			k_sleep(K_MSEC(60));
			payload.operation = CHNG_PARAMS_ACK;
			lora_send(lora_dev, payload_ptr, sizeof(payload));
			k_sleep(K_MSEC(20));
			LOG_INF("ACK sent.");
			operation = RECEIVE;
			break;

		case RANGING_SLAVE_MODE:
			LOG_INF("Rangin Mode ON.");
			lora_setup_ranging(lora_dev, &config, RangignAddress, ROLE_RECEIVER);
			lora_receive_ranging(lora_dev, &config, RangignAddress, K_FOREVER);
			ranging_done = true;
			operation = RECEIVE;
			break;

		default:
			operation = RECEIVE;
		}
#endif
	}
}
