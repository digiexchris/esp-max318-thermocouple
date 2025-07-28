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
		.clock_speed_hz = (1000000), // 1 Mhz

		.spics_io_num = -1, // Manually Control CS
		.flags = 0,
		.queue_size = 1,
	};

	MAX31856::MAX31856(gpio_num_t aCsPin, spi_host_device_t aHostId, const spi_device_interface_config_t &aDeviceConfig)
		: MAX318_Base(aCsPin, aHostId, aDeviceConfig)
	{

		// // Set the MASK to 0xFF to disable all fault detection to test spi communication
		writeRegister(MAX31856_MASK_REG, MAX31856_FAULT_ALL);

		// // Validate device communication by reading a known register
		uint8_t mask_reg = readRegister(MAX31856_MASK_REG);
		if (mask_reg != MAX31856_FAULT_ALL)
		{ // All bits set might indicate no communication
			ESP_LOGW(TAG, "Warning: setup returned unexpected values %02X, check SPI connection", mask_reg);
		}

		// turn them all back on again
		writeRegister(MAX31856_MASK_REG, MAX31856_FAULT_NONE);

		// Clear any existing faults
		uint8_t cr0_val = readRegister(MAX31856_CR0_REG);
		cr0_val |= MAX31856_CR0_FAULTCLR; // Set the FAULTCLR bit
		writeRegister(MAX31856_CR0_REG, cr0_val);

		setType(ThermocoupleType::MAX31856_TCTYPE_K); // Default to K type thermocouple

		// Enable autoconvert mode
		cr0_val = readRegister(MAX31856_CR0_REG);
		cr0_val |= MAX31856_CR0_AUTOCONVERT; // Enable auto-convert mode
		cr0_val &= ~MAX31856_CR0_1SHOT;		 // Disable one-shot mode
		cr0_val &= ~MAX31856_CR0_CJ;		 // Clear CJ bit = use internal CJ
		writeRegister(MAX31856_CR0_REG, cr0_val);

		// set default temp thresholds for the default tc type
		// setTempFaultThreshholds(-270.0f, 1372.0f);
		// setColdJunctionFaultThreshholds(-64.0f, 125.0f);

		ESP_LOGI(TAG, "MAX31856 initialized successfully on CS pin %d", aCsPin);
	}

	void MAX31856::setTempFaultThreshholds(float low, float high)
	{
		// Convert Celsius to linearized temperature fault threshold format
		// Per MAX31856 datasheet: Linearized temperature fault thresholds use 0.0625°C per LSB
		// To convert from °C to register value: divide by 0.0625 = multiply by 16
		int16_t low_int = (int16_t)(low * 16.0f);
		int16_t high_int = (int16_t)(high * 16.0f);

		// Write high threshold (big-endian)
		writeRegister(MAX31856_LTHFTH_REG, high_int >> 8);
		writeRegister(MAX31856_LTHFTL_REG, high_int & 0xFF);

		// Write low threshold (big-endian)
		writeRegister(MAX31856_LTLFTH_REG, low_int >> 8);
		writeRegister(MAX31856_LTLFTL_REG, low_int & 0xFF);
	}

	void MAX31856::setFaultMask(uint8_t aMask)
	{
		writeRegister(MAX31856_MASK_REG, aMask);
	}

	uint8_t MAX31856::readFaultMask()
	{
		return readRegister(MAX31856_MASK_REG);
	}

	void MAX31856::read(Result &anOutResult)
	{
		// oneshotTemperature();
		uint16_t cj_temp = readRegister16(MAX31856_CJTH_REG);
		// Cold junction is 14-bit signed value in bits 15:2
		int16_t cj_signed = cj_temp >> 2; // Shift to get 14-bit value
		if (cj_signed & 0x2000)
		{						 // Check bit 13 (sign bit)
			cj_signed |= 0xC000; // Sign extend bits 15:14
		}
		float cj_temp_float = cj_signed * 0.015625f; // 0.015625°C per LSB
		anOutResult.coldjunction_c = cj_temp_float;
		anOutResult.coldjunction_f = (1.8 * cj_temp_float) + 32.0;

		uint32_t tc_temp = readRegister24(MAX31856_LTCBH_REG);
		tc_temp >>= 5; // bottom 5 bits are unused, now have 19-bit value

		// Handle sign extension for 19-bit signed value
		int32_t tc_signed = tc_temp;
		if (tc_signed & 0x40000)
		{							 // Check bit 18 (sign bit for 19-bit)
			tc_signed |= 0xFFF80000; // Sign extend bits 31:19
		}
		float tc_temp_float = tc_signed * 0.0078125f; // 0.0078125°C per LSB
		anOutResult.thermocouple_c = tc_temp_float;
		anOutResult.thermocouple_f = (1.8 * tc_temp_float) + 32.0;

		uint8_t fault_val = readRegister(MAX31856_SR_REG);

		anOutResult.fault_bits = fault_val;
		anOutResult.fault.clear();

		// Log faults if requested and if any faults exist
		if (fault_val)
		{
			// Log all faults including range info
			if (fault_val & MAX31856_FAULT_CJRANGE)
				anOutResult.fault.push_back("Cold Junction Range");
			if (fault_val & MAX31856_FAULT_TCRANGE)
				anOutResult.fault.push_back("Thermocouple Range");
			if (fault_val & MAX31856_FAULT_CJHIGH)
				anOutResult.fault.push_back("Cold Junction High");
			if (fault_val & MAX31856_FAULT_CJLOW)
				anOutResult.fault.push_back("Cold Junction Low");
			if (fault_val & MAX31856_FAULT_TCHIGH)
				anOutResult.fault.push_back("Thermocouple High");
			if (fault_val & MAX31856_FAULT_TCLOW)
				anOutResult.fault.push_back("Thermocouple Low");
			if (fault_val & MAX31856_FAULT_OVUV)
				anOutResult.fault.push_back("Over/Under Voltage");
			if (fault_val & MAX31856_FAULT_OPEN)
				anOutResult.fault.push_back("Thermocouple Open");

			// Clear the faults
			uint8_t cr0_val = readRegister(MAX31856_CR0_REG);
			cr0_val |= MAX31856_CR0_FAULTCLR; // Set the FAULTCLR bit
			writeRegister(MAX31856_CR0_REG, cr0_val);
		}
	}

	void MAX31856::setColdJunctionOffset(float anOffsetCelsius)
	{
		// CJTO register: signed 8-bit, 0.0625°C per LSB
		if (anOffsetCelsius < -8.0f)
			anOffsetCelsius = -8.0f;
		if (anOffsetCelsius > 7.9375f)
			anOffsetCelsius = 7.9375f;

		int8_t offset_int = static_cast<int8_t>(anOffsetCelsius / 0.0625f);
		writeRegister(MAX31856_CJTO_REG, static_cast<uint8_t>(offset_int));
	}

	void MAX31856::setType(ThermocoupleType type)
	{
		uint8_t val = readRegister(MAX31856_CR1_REG);
		val &= 0xF0; // Mask off bottom 4 bits
		val |= static_cast<uint8_t>(type) & 0x0F;
		writeRegister(MAX31856_CR1_REG, val);
	}

	void MAX31856::setAveragingMode(uint8_t anAveragingMode)
	{
		uint8_t val = readRegister(MAX31856_CR1_REG);
		val &= 0x0F;					 // Mask off top 4 bits
		val |= (anAveragingMode & 0xF0); // Set new averaging mode
		writeRegister(MAX31856_CR1_REG, val);
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
		// MAX31856 cold junction fault thresholds: -128°C to +127°C, 1°C per LSB
		if (low < -64.0f)
			low = -64.0f;
		if (low > 125.0f)
			low = 125.0f;
		if (high < -64.0f)
			high = -64.0f;
		if (high > 125.0f)
			high = 125.0f;

		int8_t low_int = static_cast<int8_t>(low);
		int8_t high_int = static_cast<int8_t>(high);

		// Write directly as unsigned bytes (two's complement representation)
		writeRegister(MAX31856_CJHF_REG, static_cast<uint8_t>(high_int));
		writeRegister(MAX31856_CJLF_REG, static_cast<uint8_t>(low_int));

		// Log the actual values being written
		ESP_LOGI(TAG, "Cold Junction Thresholds set: Low=%.0f°C (0x%02X), High=%.0f°C (0x%02X)",
				 static_cast<double>(low), static_cast<uint8_t>(low_int),
				 static_cast<double>(high), static_cast<uint8_t>(high_int));
	}

} // namespace MAX31856