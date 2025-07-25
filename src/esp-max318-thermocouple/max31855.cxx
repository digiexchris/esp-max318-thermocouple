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
		.clock_speed_hz = (APB_CLK_FREQ / 10), // 8 Mhz

		.spics_io_num = -1, // Manually Control CS
		.flags = 0,
		.queue_size = 1,
	};

	MAX31855::MAX31855(gpio_num_t aCsPin, spi_host_device_t aHostId, const spi_device_interface_config_t &aDeviceConfig)
		: MAX318_Base(aCsPin, aHostId, aDeviceConfig)
	{
	}

	uint8_t MAX31855::readFault(bool log_fault)
	{
		ESP_LOGE("MAX31855::readFault", "NOT WORKING YET");
		abort();

		// Read the fault status register
		uint8_t fault_val = readRegister(MAX31855_SR_REG);

		// Log faults if requested and if any faults exist
		if (log_fault && fault_val)
		{
			// Log all faults including range info
			if (fault_val & MAX31855_FAULT_CJRANGE)
				ESP_LOGW(MAX31855TAG, "Fault: Cold Junction Range");
			if (fault_val & MAX31855_FAULT_TCRANGE)
				ESP_LOGW(MAX31855TAG, "Fault: Thermocouple Range");
			if (fault_val & MAX31855_FAULT_CJHIGH)
				ESP_LOGW(MAX31855TAG, "Fault: Cold Junction High");
			if (fault_val & MAX31855_FAULT_CJLOW)
				ESP_LOGW(MAX31855TAG, "Fault: Cold Junction Low");
			if (fault_val & MAX31855_FAULT_TCHIGH)
				ESP_LOGW(MAX31855TAG, "Fault: Thermocouple High");
			if (fault_val & MAX31855_FAULT_TCLOW)
				ESP_LOGW(MAX31855TAG, "Fault: Thermocouple Low");
			if (fault_val & MAX31855_FAULT_OVUV)
				ESP_LOGW(MAX31855TAG, "Fault: Over/Under Voltage");
			if (fault_val & MAX31855_FAULT_OPEN)
				ESP_LOGW(MAX31855TAG, "Fault: Thermocouple Open");
		}

		// Return the full fault value
		return fault_val;
	}

	void MAX31855::read(Result &anOutResult)
	{

		/*
		todo:
		 according to the datasheet: Note that the
MAX31855 assumes a linear relationship between temperature and voltage. Because all thermocouples exhibit
some level of nonlinearity, apply appropriate correction to
the device’s output data.
		*/

		uint32_t v = readRegister32();
		uint32_t t = v;
		uint32_t cj = v;

		uint8_t err = v & 0x7;

		if (t & 0x80000000)
		{
			// Negative value, drop the lower 18 bits and explicitly extend sign bits.
			t = 0xFFFFC000 | ((t >> 18) & 0x00003FFF);
		}
		else
		{
			// Positive value, just drop the lower 18 bits.
			t >>= 18;
		}
		// Serial.println(v, HEX);

		anOutResult.thermocouple_c = t;

		// LSB = 0.25 degrees C
		anOutResult.thermocouple_c *= 0.25;

		// ignore bottom 4 bits - they're just thermocouple data
		cj >>= 4;

		// pull the bottom 11 bits off
		float coldJunction = cj & 0x7FF;
		// check sign bit!
		if (cj & 0x800)
		{
			// Convert to negative value by extending sign and casting to signed type.
			int16_t tmp = 0xF800 | (cj & 0x7FF);
			coldJunction = tmp;
		}
		coldJunction *= 0.0625; // LSB = 0.0625 degrees

		anOutResult.coldjunction_c = coldJunction;

		anOutResult.thermocouple_f = (1.8 * anOutResult.thermocouple_f) + 32.0;
	}

} // namespace MAX31856