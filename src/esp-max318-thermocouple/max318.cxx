#include "sdkconfig.h"
#include <stdio.h>
#include <esp_log.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "soc/gpio_struct.h"
#include <memory>
#include <string.h>

#include "esp-max318-thermocouple/max318.hxx"

namespace ESP_MAX318_THERMOCOUPLE
{
	MAX318_Base::MAX318_Base(spi_device_interface_config_t aSpiDeviceConfig)
		: mySpiDeviceHandle(nullptr)
	{
		assert(aSpiDeviceConfig.spics_io_num >= 0); // CS pin must be set
	}

	void MAX318_Base::writeRegister(uint8_t address, uint8_t data, int &anOutError)
	{
		esp_err_t ret;
		anOutError = ESP_OK;
		spi_transaction_t spi_transaction = {};

		// Single transaction: address + data
		uint8_t tx_data[2] = {static_cast<uint8_t>(address | 0x80), data};

		spi_transaction.flags = SPI_TRANS_USE_RXDATA; // Can use for ≤4 bytes
		spi_transaction.length = 16;				  // 2 bytes * 8 bits
		spi_transaction.tx_buffer = tx_data;

		ret = spi_device_transmit(mySpiDeviceHandle, &spi_transaction);
		if (ret != ESP_OK)
		{
			anOutError = ret;
			return;
		}
	}

	uint8_t MAX318_Base::readRegister(uint8_t address, int &anOutError)
	{
		esp_err_t ret;
		anOutError = ESP_OK;

		// Single transaction: address + dummy byte to get data
		uint8_t tx_data[2] = {static_cast<uint8_t>(address & 0x7F), 0xFF};
		uint8_t rx_data[2];
		spi_transaction_t spi_transaction = {.flags = 0,
											 .cmd = 0,	   // No command
											 .addr = 0,	   // No address
											 .length = 16, // 2 bytes * 8 bits
														   //  .rx_length = 16, // 2 bytes * 8 bits
											 .rxlength = 16, // 2 bytes * 8 bits				   		
											 .user = NULL,
											 .tx_buffer = tx_data,
											 .rx_buffer = rx_data};

		ret = spi_device_transmit(mySpiDeviceHandle, &spi_transaction);
		if (ret != ESP_OK)
		{
			anOutError = ret;
			return 0;
		}

		// Data comes back in rx_data[1] (rx_data[0] is garbage during address)
		uint8_t reg_value = rx_data[1];
		return reg_value;
	}

	uint16_t MAX318_Base::readRegister16(uint8_t address, int &anOutError)
	{
		esp_err_t ret;
		anOutError = ESP_OK;

		// Single transaction: address + 2 data bytes
		uint8_t tx_data[3] = {static_cast<uint8_t>(address & 0x7F), 0xFF, 0xFF};
		uint8_t rx_data[3];
		spi_transaction_t spi_transaction = {
			.flags = 0,
			.cmd = 0,  // No command
			.addr = 0, // No address
			.length = 24,
			.rxlength = 24,
			.user = NULL,
			.tx_buffer = tx_data,
			.rx_buffer = rx_data,
		};

		ret = spi_device_transmit(mySpiDeviceHandle, &spi_transaction);
		if (ret != ESP_OK)
		{
			anOutError = ret;
			return 0;
		}

		// rx_data[0] is garbage during address transmission
		// rx_data[1] is first data byte, rx_data[2] is second data byte
		uint16_t reg_value = (rx_data[1] << 8) | rx_data[2];
		return reg_value;
	}

	uint32_t MAX318_Base::readRegister24(uint8_t address, int &anOutError)
	{
		anOutError = ESP_OK;

		esp_err_t ret;

		// Single transaction: address + 3 data bytes
		uint8_t tx_data[4] = {static_cast<uint8_t>(address & 0x7F), 0xFF, 0xFF, 0xFF};
		uint8_t rx_data[4];
		spi_transaction_t spi_transaction = {
			.flags = 0,
			.cmd = 0,  // No command
			.addr = 0, // No address
			.length = 32,
			.rxlength = 32,
			.user = NULL,
			.tx_buffer = tx_data,
			.rx_buffer = rx_data,
		};

		ret = spi_device_transmit(mySpiDeviceHandle, &spi_transaction);
		if (ret != ESP_OK)
		{
			anOutError = ret;
			return 0;
		}

		// rx_data[0] is garbage, rx_data[1-3] are the 3 data bytes
		uint32_t reg_value = (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3];
		return reg_value;
	}

}