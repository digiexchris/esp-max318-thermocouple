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
	MAX318_Base::MAX318_Base(gpio_num_t aCsPin, spi_host_device_t aHostId, const spi_device_interface_config_t &aDeviceConfig) : myCsPin(aCsPin), myHostId(aHostId)
	{

		devcfg = aDeviceConfig;

		// Initialize the CS pin
		gpio_config_t io_conf;
		io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
		io_conf.intr_type = GPIO_INTR_DISABLE;
		io_conf.pin_bit_mask = (1ULL << this->myCsPin);
		io_conf.mode = GPIO_MODE_OUTPUT;
		io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
		gpio_config(&io_conf);
		gpio_set_level(this->myCsPin, 1);
		vTaskDelay(100 / portTICK_PERIOD_MS);

		esp_err_t ret = spi_bus_add_device(myHostId, &devcfg, &this->mySpiDeviceHandle);
		ESP_ERROR_CHECK(ret);
	}

	void MAX318_Base::writeRegister(uint8_t address, uint8_t data)
	{
		esp_err_t ret;
		spi_transaction_t spi_transaction;
		memset(&spi_transaction, 0, sizeof(spi_transaction_t));
		uint8_t tx_data[1] = {static_cast<uint8_t>(address | 0x80)};

		gpio_set_level(this->myCsPin, 0);
		spi_transaction.flags = SPI_TRANS_USE_RXDATA;
		spi_transaction.length = 8;
		spi_transaction.tx_buffer = tx_data;
		ret = spi_device_transmit(mySpiDeviceHandle, &spi_transaction);
		ESP_ERROR_CHECK(ret);

		tx_data[0] = data;
		ret = spi_device_transmit(mySpiDeviceHandle, &spi_transaction);
		ESP_ERROR_CHECK(ret);
		gpio_set_level(this->myCsPin, 1);
	}

	uint8_t MAX318_Base::readRegister(uint8_t address)
	{
		esp_err_t ret;
		spi_transaction_t spi_transaction;
		memset(&spi_transaction, 0, sizeof(spi_transaction_t));
		uint8_t tx_data[1] = {static_cast<uint8_t>(address & 0x7F)};

		gpio_set_level(this->myCsPin, 0);
		spi_transaction.flags = SPI_TRANS_USE_RXDATA;
		spi_transaction.length = 8;
		spi_transaction.tx_buffer = tx_data;
		ret = spi_device_transmit(mySpiDeviceHandle, &spi_transaction);
		ESP_ERROR_CHECK(ret);

		tx_data[0] = 0xFF;
		ret = spi_device_transmit(mySpiDeviceHandle, &spi_transaction);
		ESP_ERROR_CHECK(ret);
		gpio_set_level(this->myCsPin, 1);
		uint8_t reg_value = spi_transaction.rx_data[0];
		return reg_value;
	}

	uint8_t MAX318_Base::readFastRegister(uint8_t address)
	{
		esp_err_t ret;
		spi_transaction_t spi_transaction;
		memset(&spi_transaction, 0, sizeof(spi_transaction_t));
		uint8_t tx_data[2] = {static_cast<uint8_t>(address & 0x7F), 0xFF};

		gpio_set_level(this->myCsPin, 0);
		spi_transaction.flags = SPI_TRANS_USE_RXDATA;
		spi_transaction.length = 16;
		spi_transaction.tx_buffer = tx_data;
		ret = spi_device_transmit(mySpiDeviceHandle, &spi_transaction);
		ESP_ERROR_CHECK(ret);
		gpio_set_level(this->myCsPin, 1);
		uint8_t reg_value = spi_transaction.rx_data[0];
		return reg_value;
	}

	uint16_t MAX318_Base::readRegister16(uint8_t address)
	{
		esp_err_t ret;
		spi_transaction_t spi_transaction;
		memset(&spi_transaction, 0, sizeof(spi_transaction_t));
		uint8_t tx_data[1] = {static_cast<uint8_t>(address & 0x7F)};

		gpio_set_level(this->myCsPin, 0);
		spi_transaction.length = 8;
		spi_transaction.flags = SPI_TRANS_USE_RXDATA;
		spi_transaction.tx_buffer = tx_data;
		ret = spi_device_transmit(mySpiDeviceHandle, &spi_transaction);
		ESP_ERROR_CHECK(ret);

		tx_data[0] = 0xFF;
		spi_transaction.length = 8;
		ret = spi_device_transmit(mySpiDeviceHandle, &spi_transaction);
		ESP_ERROR_CHECK(ret);
		uint8_t b1 = spi_transaction.rx_data[0];

		spi_transaction.length = 8;
		ret = spi_device_transmit(mySpiDeviceHandle, &spi_transaction);
		ESP_ERROR_CHECK(ret);
		uint8_t b2 = spi_transaction.rx_data[0];
		gpio_set_level(this->myCsPin, 1);

		uint16_t reg_value = ((b1 << 8) | b2);
		return reg_value;
	}

	uint32_t MAX318_Base::readRegister24(uint8_t address)
	{
		esp_err_t ret;
		spi_transaction_t spi_transaction;
		memset(&spi_transaction, 0, sizeof(spi_transaction_t));
		uint8_t tx_data[1] = {static_cast<uint8_t>(address & 0x7F)};

		gpio_set_level(this->myCsPin, 0);
		spi_transaction.length = 8;
		spi_transaction.flags = SPI_TRANS_USE_RXDATA;
		spi_transaction.tx_buffer = tx_data;
		ret = spi_device_transmit(mySpiDeviceHandle, &spi_transaction);
		ESP_ERROR_CHECK(ret);

		tx_data[0] = 0xFF;
		spi_transaction.length = 8;
		ret = spi_device_transmit(mySpiDeviceHandle, &spi_transaction);
		ESP_ERROR_CHECK(ret);
		uint8_t b1 = spi_transaction.rx_data[0];

		tx_data[0] = 0xFF;
		spi_transaction.length = 8;
		ret = spi_device_transmit(mySpiDeviceHandle, &spi_transaction);
		ESP_ERROR_CHECK(ret);
		uint8_t b2 = spi_transaction.rx_data[0];

		tx_data[0] = 0xFF;
		spi_transaction.length = 8;
		ret = spi_device_transmit(mySpiDeviceHandle, &spi_transaction);
		ESP_ERROR_CHECK(ret);
		uint8_t b3 = spi_transaction.rx_data[0];
		gpio_set_level(this->myCsPin, 0);

		uint32_t reg_value = ((b1 << 16) | (b2 << 8) | b3);
		return reg_value;
	}

}