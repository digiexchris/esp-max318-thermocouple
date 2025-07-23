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

#include "esp-max318-thermocouple/max31856.hxx"

namespace ESP_MAX318_THERMOCOUPLE
{

	const char *TAG = "MAX31856";

	const spi_device_interface_config_t MAX31856::defaultSpiDeviceConfig = {

		.dummy_bits = 0,
		.mode = 1,
		.clock_speed_hz = (APB_CLK_FREQ / 10), // 8 Mhz

		.spics_io_num = -1, // Manually Control CS
		.flags = 0,
		.queue_size = 1,
	};

	MAX31856::MAX31856(gpio_num_t aCsPin, spi_host_device_t aHostId, const spi_device_interface_config_t &aDeviceConfig)
		: MAX318_Base(aCsPin, aHostId, aDeviceConfig)
	{
		// Set the MASK to 0x00 to enable all fault detection
		writeRegister(MAX31856_MASK_REG, 0xFF);

		// Clear any existing faults
		// uint8_t cr0_val = readRegister(MAX31856_CR0_REG);
		// cr0_val |= MAX31856_CR0_FAULTCLR; // Set the FAULTCLR bit
		// writeRegister(MAX31856_CR0_REG, cr0_val);

		// Open Circuit Detection
		// writeRegister(MAX31856_CR0_REG, MAX31856_CR0_OCFAULT0);

		// Set wide temperature fault thresholds to avoid false triggers
		// setColdJunctionFaultThreshholds(-40, 140); // Wide range for Cold Junction
		// setTempFaultThreshholds(-40, 1360);		   // Wide range for Thermocouple

		// Log the fault status after initialization to verify if our fix worked
		// uint8_t fault = readRegister(MAX31856_SR_REG, anIndex);
		// ESP_LOGI(TAG, "Fault register after setup: 0x%02X", fault, anIndex);

		setType(ThermocoupleType::MAX31856_TCTYPE_K); // Default to K type thermocouple
	}

	void MAX31856::setTempFaultThreshholds(float low, float high)
	{
		ESP_ERROR_CHECK(false);
		low *= 16;
		high *= 16;
		int16_t low_int = low;
		int16_t high_int = high;
		writeRegister(MAX31856_LTHFTH_REG, high_int >> 8);
		writeRegister(MAX31856_LTHFTL_REG, high_int);
		writeRegister(MAX31856_LTLFTH_REG, low_int >> 8);
		writeRegister(MAX31856_LTLFTL_REG, low_int);
	}

	uint8_t MAX31856::readFault(bool log_fault)
	{
		ESP_LOGE("MAX31856::readFault", "NOT WORKING YET");
		abort();

		// Read the fault status register
		uint8_t fault_val = readRegister(MAX31856_SR_REG);

		// Log faults if requested and if any faults exist
		if (log_fault && fault_val)
		{
			// Log all faults including range info
			if (fault_val & MAX31856_FAULT_CJRANGE)
				ESP_LOGW(TAG, "Fault: Cold Junction Range");
			if (fault_val & MAX31856_FAULT_TCRANGE)
				ESP_LOGW(TAG, "Fault: Thermocouple Range");
			if (fault_val & MAX31856_FAULT_CJHIGH)
				ESP_LOGW(TAG, "Fault: Cold Junction High");
			if (fault_val & MAX31856_FAULT_CJLOW)
				ESP_LOGW(TAG, "Fault: Cold Junction Low");
			if (fault_val & MAX31856_FAULT_TCHIGH)
				ESP_LOGW(TAG, "Fault: Thermocouple High");
			if (fault_val & MAX31856_FAULT_TCLOW)
				ESP_LOGW(TAG, "Fault: Thermocouple Low");
			if (fault_val & MAX31856_FAULT_OVUV)
				ESP_LOGW(TAG, "Fault: Over/Under Voltage");
			if (fault_val & MAX31856_FAULT_OPEN)
				ESP_LOGW(TAG, "Fault: Thermocouple Open");

			// Clear the faults
			uint8_t cr0_val = readRegister(MAX31856_CR0_REG);
			cr0_val |= MAX31856_CR0_FAULTCLR; // Set the FAULTCLR bit
			writeRegister(MAX31856_CR0_REG, cr0_val);

			// Read back the fault register to see if they were cleared
			uint8_t cleared = readRegister(MAX31856_SR_REG);
			if (cleared)
			{
				ESP_LOGI(TAG, "After clear attempt, fault register: 0x%02X", cleared);
			}
		}

		// Return the full fault value
		return fault_val;
	}

	void MAX31856::read(Result &anOutResult)
	{
		oneshotTemperature();
		uint16_t cj_temp = readRegister16(MAX31856_CJTH_REG);
		float cj_temp_float = cj_temp;
		cj_temp_float /= 256.0;
		anOutResult.coldjunction_c = cj_temp_float;
		anOutResult.coldjunction_f = (1.8 * cj_temp_float) + 32.0;

		uint32_t tc_temp = readRegister24(MAX31856_LTCBH_REG);
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

	void MAX31856::setType(ThermocoupleType type)
	{
		uint8_t val = readRegister(MAX31856_CR1_REG);
		val &= 0xF0; // Mask off bottom 4 bits
		val |= static_cast<uint8_t>(type) & 0x0F;
		writeRegister(MAX31856_CR1_REG, val);
	}

	void MAX31856::oneshotTemperature()
	{
		writeRegister(MAX31856_CJTO_REG, 0x00);
		uint8_t val = readRegister(MAX31856_CR0_REG);
		val &= ~MAX31856_CR0_AUTOCONVERT;
		val |= MAX31856_CR0_1SHOT;
		writeRegister(MAX31856_CR0_REG, val);
		vTaskDelay(250 / portTICK_PERIOD_MS);
	}

	MAX31856::ThermocoupleType MAX31856::getType()
	{
		uint8_t val = readRegister(MAX31856_CR1_REG);

		val &= 0x0F;

		ThermocoupleType type = static_cast<ThermocoupleType>(val);

		switch (type)
		{
		case ThermocoupleType::MAX31856_TCTYPE_B:
			ESP_LOGI(TAG, "TC Type: B");
			break;
		case ThermocoupleType::MAX31856_TCTYPE_E:
			ESP_LOGI(TAG, "TC Type: E");
			break;
		case ThermocoupleType::MAX31856_TCTYPE_J:
			ESP_LOGI(TAG, "TC Type: J");
			break;
		case ThermocoupleType::MAX31856_TCTYPE_K:
			ESP_LOGI(TAG, "TC Type: K");
			break;
		case ThermocoupleType::MAX31856_TCTYPE_N:
			ESP_LOGI(TAG, "TC Type: N");
			break;
		case ThermocoupleType::MAX31856_TCTYPE_R:
			ESP_LOGI(TAG, "TC Type: R");
			break;
		case ThermocoupleType::MAX31856_TCTYPE_S:
			ESP_LOGI(TAG, "TC Type: S");
			break;
		case ThermocoupleType::MAX31856_TCTYPE_T:
			ESP_LOGI(TAG, "TC Type: T");
			break;
		case ThermocoupleType::MAX31856_VMODE_G8:
			ESP_LOGI(TAG, "Voltage x8 Gain mode");
			break;
		case ThermocoupleType::MAX31856_VMODE_G32:
			ESP_LOGI(TAG, "Voltage x8 Gain mode");
			break;
		default:
			ESP_LOGI(TAG, "TC Type: Unknown");
			break;
		}

		return type;
	}

	void MAX31856::setColdJunctionFaultThreshholds(float low, float high)
	{
		// According to the datasheet, the cold junction temperature register has a resolution
		// of 0.0625°C per LSB (1/16), so we need to multiply our values by 16
		low *= 16;
		high *= 16;

		// Convert to integers
		int16_t low_int = static_cast<int16_t>(low);
		int16_t high_int = static_cast<int16_t>(high);

		// Write the high threshold - the register only takes a single byte
		// CJHF is a single byte register that takes values from -128°C to +127°C in 1°C steps
		writeRegister(MAX31856_CJHF_REG, static_cast<uint8_t>(high_int / 16));

		// Write the low threshold - the register only takes a single byte
		// CJLF is a single byte register that takes values from -128°C to +127°C in 1°C steps
		writeRegister(MAX31856_CJLF_REG, static_cast<uint8_t>(low_int / 16));

		// Log the settings
		ESP_LOGI(TAG, "Cold Junction Thresholds set: Low=%0.2f°C, High=%0.2f°C",
				 static_cast<float>(low_int) / 16.0, static_cast<float>(high_int) / 16.0);
	}

} // namespace MAX31856