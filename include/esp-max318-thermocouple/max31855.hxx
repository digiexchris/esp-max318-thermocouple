#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "soc/gpio_struct.h"

#include <esp_log.h>

#include "esp-max318-thermocouple/max318.hxx"

namespace ESP_MAX318_THERMOCOUPLE
{

	constexpr uint8_t MAX31855_CR1_REG = 0x01;
	constexpr uint8_t MAX31855_MASK_REG = 0x02;
	constexpr uint8_t MAX31855_CJHF_REG = 0x03;
	constexpr uint8_t MAX31855_CJLF_REG = 0x04;
	constexpr uint8_t MAX31855_LTHFTH_REG = 0x05;
	constexpr uint8_t MAX31855_LTHFTL_REG = 0x06;
	constexpr uint8_t MAX31855_LTLFTH_REG = 0x07;
	constexpr uint8_t MAX31855_LTLFTL_REG = 0x08;
	constexpr uint8_t MAX31855_CJTO_REG = 0x09;
	constexpr uint8_t MAX31855_CJTH_REG = 0x0A;
	constexpr uint8_t MAX31855_CJTL_REG = 0x0B;
	constexpr uint8_t MAX31855_LTCBH_REG = 0x0C;
	constexpr uint8_t MAX31855_LTCBM_REG = 0x0D;
	constexpr uint8_t MAX31855_LTCBL_REG = 0x0E;
	constexpr uint8_t MAX31855_SR_REG = 0x0F;

	constexpr uint8_t MAX31855_FAULT_NONE = 0x00;	   ///< Disable all fault checks
	constexpr uint8_t MAX31855_FAULT_OPEN = 0x01;	   ///< Enable open circuit fault check
	constexpr uint8_t MAX31855_FAULT_SHORT_GND = 0x02; ///< Enable short to GND fault check
	constexpr uint8_t MAX31855_FAULT_SHORT_VCC = 0x04; ///< Enable short to VCC fault check
	constexpr uint8_t MAX31855_FAULT_ALL = 0x07;	   ///< Enable all fault checks

	using MAX31855Callback = void (*)(Result *result);

	class MAX31855 : public MAX318_Base
	{
	public:
		MAX31855(gpio_num_t aCsPin, spi_host_device_t aHostId, const spi_device_interface_config_t &aDeviceConfig = defaultSpiDeviceConfig);

		bool read(Result &anOutResult) override;
		bool setTempFaultThreshholds(float aLow, float aHigh, int &anOutError) override;
		bool setColdJunctionFaultThreshholds(float aLow, float aHigh, int &anOutError) override;

		static const spi_device_interface_config_t defaultSpiDeviceConfig;

	private:
		uint32_t readRegister32(int &anOutError);
		double myTemperatureHighThreshold = 1372.0;
		double myTemperatureLowThreshold = -270.0;
		double myColdJunctionHighThreshold = 125.0;
		double myColdJunctionLowThreshold = -55.0;
	};

} // namespace MAX31855