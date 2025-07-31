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

	MAX31856::MAX31856(spi_device_interface_config_t aSpiDeviceConfig)
		: MAX318_Base(aSpiDeviceConfig)
	{
		assert(aSpiDeviceConfig.mode == 1); // Mode 1 is required by the MAX31856
	}

	bool MAX31856::configure(const MAX318Config *aConfig, const spi_device_handle_t &aHandle)
	{
		myConfig = *static_cast<const MAX31856Config *>(aConfig);
		mySpiDeviceHandle = aHandle;

		int error = 0;

		// // Set the MASK to 0xFF to disable all fault detection to test spi communication
		writeRegister(MAX31856_MASK_REG, MAX31856_FAULT_ALL, error);
		assert(error == 0);

		// // Validate device communication by reading a known register
		uint8_t mask_reg = readRegister(MAX31856_MASK_REG, error);
		assert(error == 0);
		if (mask_reg != MAX31856_FAULT_ALL)
		{ // All bits set might indicate no communication
			ESP_LOGW(TAG, "Warning: setup returned unexpected values %02X, check SPI connection", mask_reg);
		}

		// turn them all back on again
		writeRegister(MAX31856_MASK_REG, myConfig.fault_mask, error);
		assert(error == 0);

		// Clear any existing faults
		uint8_t cr0_val = readRegister(MAX31856_CR0_REG, error);
		assert(error == 0);
		cr0_val |= MAX31856_CR0_FAULTCLR; // Set the FAULTCLR bit
		writeRegister(MAX31856_CR0_REG, cr0_val, error);
		assert(error == 0);

		setType(ThermocoupleType::MAX31856_TCTYPE_K, error); // Default to K type thermocouple
		assert(error == 0);

		// The device should be in one-shot mode in order to set averaging samples
		setConvertType(false, error);
		assert(error == 0);

		setAveragingMode(myConfig.averaging_samples, error);
		assert(error == 0);

		if (myConfig.auto_convert)
		{
			// If auto convert mode is enabled, set the device to auto convert
			setConvertType(true, error);
			assert(error == 0);
		}

		// Ensure internal CJ temp sensor is enabled
		cr0_val = readRegister(MAX31856_CR0_REG, error);
		assert(error == 0);
		cr0_val &= ~MAX31856_CR0_CJ; // Clear CJ bit = use internal CJ
		writeRegister(MAX31856_CR0_REG, cr0_val, error);
		assert(error == 0);

		// set default temp thresholds for the tc type
		setTempFaultThreshholds(myConfig.temp_fault_low, myConfig.temp_fault_high, error);
		setColdJunctionFaultThreshholds(myConfig.cold_junction_fault_high, myConfig.cold_junction_fault_low, error);

		return true;
	}

	bool MAX31856::setConvertType(bool isAutoConvert, int &anOutError)
	{
		esp_err_t ret;
		anOutError = ESP_OK;

		uint8_t cr0_val = readRegister(MAX31856_CR0_REG, ret);
		if (ret != ESP_OK)
		{
			anOutError = ret;
			return false;
		}

		if (isAutoConvert)
		{
			cr0_val |= MAX31856_CR0_AUTOCONVERT; // Enable auto-convert mode
			cr0_val &= ~MAX31856_CR0_1SHOT;		 // Disable one-shot mode
		}
		else
		{
			cr0_val &= ~MAX31856_CR0_AUTOCONVERT; // Disable auto-convert mode
			cr0_val |= MAX31856_CR0_1SHOT;		  // Enable one-shot mode
		}

		writeRegister(MAX31856_CR0_REG, cr0_val, ret);
		if (ret != ESP_OK)
		{
			anOutError = ret;
			return false;
		}

		return true;
	}

	bool MAX31856::setTempFaultThreshholds(float low, float high, int &anOutError)
	{
		// Convert Celsius to linearized temperature fault threshold format
		// Per MAX31856 datasheet: Linearized temperature fault thresholds use 0.0625°C per LSB
		// To convert from °C to register value: divide by 0.0625 = multiply by 16
		int16_t low_int = (int16_t)(low * 16.0f);
		int16_t high_int = (int16_t)(high * 16.0f);

		esp_err_t ret;
		anOutError = ESP_OK;

		// Write high threshold (big-endian)
		writeRegister(MAX31856_LTHFTH_REG, high_int >> 8, ret);
		if (ret != ESP_OK)
		{
			anOutError = ret;
			return false; // Return false on error
		}
		writeRegister(MAX31856_LTHFTL_REG, high_int & 0xFF, ret);
		if (ret != ESP_OK)
		{
			anOutError = ret;
			return false; // Return false on error
		}

		// Write low threshold (big-endian)
		writeRegister(MAX31856_LTLFTH_REG, low_int >> 8, ret);
		if (ret != ESP_OK)
		{
			anOutError = ret;
			return false; // Return false on error
		}
		writeRegister(MAX31856_LTLFTL_REG, low_int & 0xFF, ret);
		if (ret != ESP_OK)
		{
			anOutError = ret;
			return false; // Return false on error
		}

		return (anOutError == ESP_OK);
	}

	bool MAX31856::setFaultMask(uint8_t aMask, int &anOutError)
	{
		writeRegister(MAX31856_MASK_REG, aMask, anOutError);

		if (anOutError != ESP_OK)
		{
			return false;
		}
		return true;
	}

	uint8_t MAX31856::readFaultMask(int &anOuterror)
	{
		return readRegister(MAX31856_MASK_REG, anOuterror);
	}

	bool MAX31856::oneShotTemperature(int &anOutError)
	{
		uint8_t val = readRegister(MAX31856_CR0_REG, anOutError);
		if (anOutError != ESP_OK)
		{
			return false; // Return on error
		}

		val &= ~MAX31856_CR0_AUTOCONVERT;
		val |= MAX31856_CR0_1SHOT;
		writeRegister(MAX31856_CR0_REG, val, anOutError);
		if (anOutError != ESP_OK)
		{
			return false; // Return on error
		}

		// Wait for conversion to complete by checking DRDY bit
		// or use timeout to prevent infinite loop
		uint32_t timeout = 0;
		const uint32_t MAX_TIMEOUT = 300; // ms
		while (timeout < MAX_TIMEOUT)
		{
			uint8_t status = readRegister(MAX31856_SR_REG, anOutError);
			if (anOutError != ESP_OK)
			{
				return false; // Return on error
			}

			if (!(status & 0x80))
			{ // DRDY bit cleared = conversion done
				break;
			}
			vTaskDelay(10 / portTICK_PERIOD_MS);
			timeout += 10;
		}

		if (timeout >= MAX_TIMEOUT)
		{
			ESP_LOGW(TAG, "One-shot conversion timeout");
			return false;
		}

		return true;
	}

	bool MAX31856::read(Result &anOutResult)
	{

		anOutResult.spi_success = true;	 // Assume valid unless we find a fault
		anOutResult.error_code = ESP_OK; // Default to no error

		esp_err_t ret;

		if (!myConfig.auto_convert)
		{
			// If not in auto convert mode, trigger a one-shot temperature read
			oneShotTemperature(ret);
			if (ret != ESP_OK)
			{
				anOutResult.spi_success = false;
				anOutResult.error_code = ret;
				return false; // Return on error
			}
		}

		uint16_t cj_temp = readRegister16(MAX31856_CJTH_REG, ret);
		if (ret != ESP_OK)
		{
			anOutResult.spi_success = false;
			anOutResult.error_code = ret;
			return false;
		}

		// Cold junction is 14-bit signed value in bits 15:2
		int16_t cj_signed = cj_temp >> 2; // Shift to get 14-bit value
		if (cj_signed & 0x2000)
		{						 // Check bit 13 (sign bit)
			cj_signed |= 0xC000; // Sign extend bits 15:14
		}
		float cj_temp_float = cj_signed * 0.015625f; // 0.015625°C per LSB
		anOutResult.coldjunction_c = cj_temp_float;
		anOutResult.coldjunction_f = (1.8 * cj_temp_float) + 32.0;

		uint32_t tc_temp = readRegister24(MAX31856_LTCBH_REG, ret);
		if (ret != ESP_OK)
		{
			anOutResult.spi_success = false;
			anOutResult.error_code = ret;
			return false;
		}

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

		uint8_t fault_val = readRegister(MAX31856_SR_REG, ret);
		if (ret != ESP_OK)
		{
			anOutResult.spi_success = false;
			anOutResult.error_code = ret;
			return false;
		}

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
			uint8_t cr0_val = readRegister(MAX31856_CR0_REG, ret);
			if (ret != ESP_OK)
			{
				anOutResult.spi_success = false;
				anOutResult.error_code = ret;
				return false;
			}

			cr0_val |= MAX31856_CR0_FAULTCLR; // Set the FAULTCLR bit
			writeRegister(MAX31856_CR0_REG, cr0_val, ret);
			if (ret != ESP_OK)
			{
				anOutResult.spi_success = false;
				anOutResult.error_code = ret;
				return false;
			}
		}

		return true;
	}

	bool MAX31856::setColdJunctionOffset(float anOffsetCelsius, int &anOutError)
	{
		// CJTO register: signed 8-bit, 0.0625°C per LSB
		if (anOffsetCelsius < -8.0f)
			anOffsetCelsius = -8.0f;
		if (anOffsetCelsius > 7.9375f)
			anOffsetCelsius = 7.9375f;

		int8_t offset_int = static_cast<int8_t>(anOffsetCelsius / 0.0625f);

		esp_err_t ret;
		anOutError = ESP_OK;
		writeRegister(MAX31856_CJTO_REG, static_cast<uint8_t>(offset_int), ret);
		if (ret != ESP_OK)
		{
			anOutError = ret;
			return false;
		}

		return true;
	}

	bool MAX31856::setType(ThermocoupleType type, int &anOutError)
	{
		esp_err_t ret;
		anOutError = ESP_OK;
		uint8_t val = readRegister(MAX31856_CR1_REG, ret);
		if (ret != ESP_OK)
		{
			anOutError = ret;
			return false;
		}
		val &= 0xF0; // Mask off bottom 4 bits
		val |= static_cast<uint8_t>(type) & 0x0F;
		writeRegister(MAX31856_CR1_REG, val, ret);
		if (ret != ESP_OK)
		{
			anOutError = ret;
			return false;
		}

		return true;
	}

	bool MAX31856::setAveragingMode(AveragingSamples aSamples, int &anOutError)
	{
		uint8_t anAveragingMode = 0;

		switch (aSamples)
		{
		case AveragingSamples::AVG_1:
			anAveragingMode = MAX31856_CR1_AVG_MODE_1;
			break;
		case AveragingSamples::AVG_2:
			anAveragingMode = MAX31856_CR1_AVG_MODE_2;
			break;
		case AveragingSamples::AVG_4:
			anAveragingMode = MAX31856_CR1_AVG_MODE_4;
			break;
		case AveragingSamples::AVG_8:
			anAveragingMode = MAX31856_CR1_AVG_MODE_8;
			break;
		case AveragingSamples::AVG_16:
			anAveragingMode = MAX31856_CR1_AVG_MODE_16;
			break;
		default:
			anOutError = ESP_ERR_INVALID_ARG; // Invalid averaging mode
			return false;
		}
		esp_err_t ret;
		uint8_t val = readRegister(MAX31856_CR1_REG, ret);
		if (ret != ESP_OK)
		{
			anOutError = ret;
			return false;
		}
		val &= 0x0F;					 // Mask off top 4 bits
		val |= (anAveragingMode & 0xF0); // Set new averaging mode
		writeRegister(MAX31856_CR1_REG, val, ret);
		if (ret != ESP_OK)
		{
			anOutError = ret;
			return false;
		}

		return true;
	}

	bool MAX31856::setColdJunctionFaultThreshholds(float low, float high, int &anOutError)
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

		esp_err_t ret;
		anOutError = ESP_OK;

		// Write directly as unsigned bytes (two's complement representation)
		writeRegister(MAX31856_CJHF_REG, static_cast<uint8_t>(high_int), ret);
		if (ret != ESP_OK)
		{
			anOutError = ret;
			return false;
		}
		writeRegister(MAX31856_CJLF_REG, static_cast<uint8_t>(low_int), ret);
		if (ret != ESP_OK)
		{
			anOutError = ret;
			return false;
		}
		return true;
	}

} // namespace MAX31856