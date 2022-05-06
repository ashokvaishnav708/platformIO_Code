
//Currently developed only for LORA 

#include <SX1280.h>
#include <SX1280_DEFINITIONS.h>

#include <drivers/lora.h>
#include<drivers/spi.h>

/*
#if defined SPI0

#elif defined SPI1

#elif defined SPI2

#elif defined SPI3

#else 

#endif

*/

#define SPI3

/*
 SPI3 is currently used by-default
i.e cs pin already configured
 -> NRESET
 -> RFBUSY
 -> DIO1
*/

struct SX1280 {
    struct device *spi_dev;
    struct gpio_dt_spec *pinCS;
    struct gpio_dt_spec *pinRESET;
    struct gpio_dt_spec *pinRFBUSY;
    struct gpio_dt_spec *pinDIO1;
};

bool beginSX1280(struct SX1280_device *dev)
{
    
}

bool writeCommandSX1280(uint8_t command, uint8_t *buffer, uint16_t size)
{
    int ret;

	const struct spi_buf buf[2] = {
		{
			.buf = &command,
			.len = sizeof(command)
		},
		{
			.buf = buffer,
			.len = size
		}
	};

	struct spi_buf_set tx = {
		.buffers = buf,
		.count = ARRAY_SIZE(buf),
	};

	ret = spi_write(dev_data.spi, &dev_data.spi_cfg, &tx);
	// printk("wc3: %x\n", ret);

	if (ret < 0) {
		return false;
	}
    else {
        return true;
    }
}

/*
void setupSX1280()
{
    setMode(MODE_STDBY_RC);
    setPacketType(PACKET_TYPE_LORA);
    setRfFrequency(Frequency, Offset);
    calibrateImage(0);
    setModulationParams(SpreadingFactor, Bandwidth, CodeRate, LDRO_AUTO);
    setBufferBaseAddress(0x00, 0x00);
    setPacketParams(8, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);
    setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);
    LORA_MAC_PUBLIC_SYNCWORD = 0x34
    setHighSensitivity();
    setDioIrqParams(IRQ_RADIO_ALL, IRQ_TX_DONE, 0, 0);
}

*/

void setOperationModeSX1280(uint8_t OPMODE)
{

}

uint8_t writeSX1280(uint8_t commands, int size)
{

}

bool writeBufferSX1280()






