#include "sdkconfig.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "soc/gpio_struct.h"

#include "esp-max318-thermocouple/max31855.hxx"

namespace ESP_MAX318_THERMOCOUPLE
{

	const char *MAX31855TAG = "MAX31855";

	const spi_device_interface_config_t MAX31855::defaultSpiDeviceConfig = {

		.dummy_bits = 0,
		.mode = 0,
		.clock_speed_hz = (1000000), // 1 Mhz

		.spics_io_num = -1, // Manually Control CS
		.flags = SPI_DEVICE_HALFDUPLEX,
		.queue_size = 1,
	};

	MAX31855::MAX31855(gpio_num_t aCsPin, spi_host_device_t aHostId, const spi_device_interface_config_t &aDeviceConfig)
		: MAX318_Base(aCsPin, aHostId, aDeviceConfig)
	{
	}

	bool MAX31855::read(Result &anOutResult)
	{

		/*
		todo:
		 according to the datasheet: Note that the
MAX31855 assumes a linear relationship between temperature and voltage. Because all thermocouples exhibit
some level of nonlinearity, apply appropriate correction to
the device’s output data.
		*/

		anOutResult.error_code = ESP_OK;
		anOutResult.spi_success = true;

		uint32_t v = readRegister32(anOutResult.error_code);
		if (anOutResult.error_code != ESP_OK)
		{
			anOutResult.spi_success = false;
			return false;
		}

		// Extract fault bits (bits 2, 1, 0)
		uint8_t fault_bits = v & 0x7;

		// Check for general fault (bit 16)
		bool general_fault = (v >> 16) & 0x1;

		// Build fault string
		anOutResult.fault.clear();
		anOutResult.fault_bits = fault_bits;
		if (general_fault)
		{
			anOutResult.fault.emplace_back("General Fault");
		}
		if (fault_bits & 0x1)
		{
			anOutResult.fault.emplace_back("Open Circuit");
		}
		if (fault_bits & 0x2)
		{
			anOutResult.fault.emplace_back("Short to GND");
		}
		if (fault_bits & 0x4)
		{
			anOutResult.fault.emplace_back("Short to VCC");
		}

		// Thermocouple temperature (bits 31-18, 14-bit signed)
		int32_t tc_raw = (v >> 18) & 0x3FFF;
		if (tc_raw & 0x2000)
		{
			// Sign extend for negative values
			tc_raw |= 0xFFFFC000;
		}
		anOutResult.thermocouple_c = tc_raw * 0.25f; // LSB = 0.25°C
		anOutResult.thermocouple_f = (anOutResult.thermocouple_c * 1.8f) + 32.0f;

		if (anOutResult.thermocouple_c > myTemperatureHighThreshold)
		{
			anOutResult.fault.emplace_back("Thermocouple Temperature High");
		}
		else if (anOutResult.thermocouple_c < myTemperatureLowThreshold)
		{
			anOutResult.fault.emplace_back("Thermocouple Temperature Low");
		}

		if (anOutResult.coldjunction_c > myColdJunctionHighThreshold)
		{
			anOutResult.fault.emplace_back("Cold Junction Temperature High");
		}
		else if (anOutResult.coldjunction_c < myColdJunctionLowThreshold)
		{
			anOutResult.fault.emplace_back("Cold Junction Temperature Low");
		}

		// Cold junction temperature (bits 15-4, 12-bit signed)
		int32_t cj_raw = (v >> 4) & 0xFFF; // 12 bits, not 11
		if (cj_raw & 0x800)
		{
			// Sign extend for negative values
			cj_raw |= 0xFFFFF000;
		}
		anOutResult.coldjunction_c = cj_raw * 0.0625f; // LSB = 0.0625°C
		anOutResult.coldjunction_f = (anOutResult.coldjunction_c * 1.8f) + 32.0f;

		return true;
	}

	uint32_t MAX31855::readRegister32(int &anOutError)
	{
		uint8_t rx_data[4];
		spi_transaction_t trans{
			.flags = 0,
			.length = 0,	// No TX data
			.rxlength = 32, // Receive 32 bits (4 bytes)
			.tx_buffer = NULL,
			.rx_buffer = rx_data};

		anOutError = spi_device_transmit(mySpiDeviceHandle, &trans);

		if (anOutError != ESP_OK)
		{
			return 0; // Return 0 on error
		}

		uint32_t result = (rx_data[0] << 24) |
						  (rx_data[1] << 16) |
						  (rx_data[2] << 8) |
						  rx_data[3];
		return result;
	}

	bool MAX31855::setTempFaultThreshholds(float aLow, float aHigh, int &anOutError)
	{
		// MAX31855 temperature fault thresholds: -270°C to +1372°C, 0.25°C per LSB
		if (aLow < -270.0f)
			aLow = -270.0f;
		if (aHigh > 1372.0f)
			aHigh = 1372.0f;

		myTemperatureLowThreshold = aLow;
		myTemperatureHighThreshold = aHigh;
		anOutError = ESP_OK;
		return true;
	}

	bool MAX31855::setColdJunctionFaultThreshholds(float aLow, float aHigh, int &anOutError)
	{
		// MAX31855 cold junction fault thresholds: -64°C to +125°C, 0.0625°C per LSB
		if (aLow < -64.0f)
			aLow = -64.0f;
		if (aHigh > 125.0f)
			aHigh = 125.0f;

		myColdJunctionLowThreshold = aLow;
		myColdJunctionHighThreshold = aHigh;

		anOutError = ESP_OK;
		return true;
	}

} // namespace ESP_MAX318_THERMOCOUPLE