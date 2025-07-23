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
		uint16_t cj_temp = readRegister16(MAX31855_CJTH_REG);
		float cj_temp_float = cj_temp;
		cj_temp_float /= 256.0;
		anOutResult.coldjunction_c = cj_temp_float;
		anOutResult.coldjunction_f = (1.8 * cj_temp_float) + 32.0;

		uint32_t tc_temp = readRegister24(MAX31855_LTCBH_REG);
		if (tc_temp & 0x800000)
		{
			tc_temp |= 0xFF000000; // fix sign bit
		}
		tc_temp >>= 5; // bottom 5 bits are unused
		float tc_temp_float = tc_temp;
		tc_temp_float *= 0.0078125;
		anOutResult.thermocouple_c = tc_temp_float;
		anOutResult.thermocouple_f = (1.8 * tc_temp_float) + 32.0;
	}

} // namespace MAX31856