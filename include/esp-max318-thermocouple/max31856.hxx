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

	using MAX31856Callback = void (*)(Result *result);

	class MAX31856 : public MAX318_Base
	{
	public:
		enum class ThermocoupleType
		{
			MAX31856_TCTYPE_B = 0b0000,
			MAX31856_TCTYPE_E = 0b0001,
			MAX31856_TCTYPE_J = 0b0010,
			MAX31856_TCTYPE_K = 0b0011,
			MAX31856_TCTYPE_N = 0b0100,
			MAX31856_TCTYPE_R = 0b0101,
			MAX31856_TCTYPE_S = 0b0110,
			MAX31856_TCTYPE_T = 0b0111,
			MAX31856_VMODE_G8 = 0b1000,
			MAX31856_VMODE_G32 = 0b1100,
		};

		MAX31856(gpio_num_t aCsPin, spi_host_device_t aHostId, const spi_device_interface_config_t &aDeviceConfig = defaultSpiDeviceConfig);

		ThermocoupleType getType();
		void setTempFaultThreshholds(float aLow, float aHigh) override;
		void setColdJunctionFaultThreshholds(float aLow, float aHigh) override;
		uint8_t readFault(bool logFault = false);
		virtual void read(Result &anOutResult) override;
		void setType(ThermocoupleType aType);
		void setColdJunctionOffset(float anOffsetCelsius);
		void setFaultMask(uint8_t aMask);
		uint8_t readFaultMask();
		void setAveragingMode(uint8_t anAveragingMode);

		static const spi_device_interface_config_t defaultSpiDeviceConfig;

		const uint8_t MAX31856_CR0_REG = 0x00;
		const uint8_t MAX31856_CR0_AUTOCONVERT = 0x80;
		const uint8_t MAX31856_CR0_1SHOT = 0x40;
		const uint8_t MAX31856_CR0_OCFAULT1 = 0x20;
		const uint8_t MAX31856_CR0_OCFAULT0 = 0x10;
		const uint8_t MAX31856_CR0_CJ = 0x08;
		const uint8_t MAX31856_CR0_FAULT = 0x04;
		const uint8_t MAX31856_CR0_FAULTCLR = 0x02;

		const uint8_t MAX31856_CR1_TC_B = 0x00;
		const uint8_t MAX31856_CR1_TC_E = 0x01;
		const uint8_t MAX31856_CR1_TC_J = 0x02;
		const uint8_t MAX31856_CR1_TC_K = 0x03;
		const uint8_t MAX31856_CR1_TC_N = 0x04;
		const uint8_t MAX31856_CR1_TC_R = 0x05;
		const uint8_t MAX31856_CR1_TC_S = 0x06;
		const uint8_t MAX31856_CR1_TC_T = 0x07;
		const uint8_t MAX31856_CR1_VMODE_G8 = 0x08;
		const uint8_t MAX31856_CR1_VMODE_G32 = 0x0C;

		const uint8_t MAX31856_CR1_AVG_MODE_1 = 0x00;  // No averaging
		const uint8_t MAX31856_CR1_AVG_MODE_2 = 0x10;  // Enable + 2 samples
		const uint8_t MAX31856_CR1_AVG_MODE_4 = 0x30;  // Enable + 4 samples
		const uint8_t MAX31856_CR1_AVG_MODE_8 = 0x50;  // Enable + 8 samples
		const uint8_t MAX31856_CR1_AVG_MODE_16 = 0x70; // Enable + 16 samples

		const uint8_t MAX31856_CR1_REG = 0x01;
		const uint8_t MAX31856_MASK_REG = 0x02;
		const uint8_t MAX31856_CJHF_REG = 0x03;
		const uint8_t MAX31856_CJLF_REG = 0x04;
		const uint8_t MAX31856_LTHFTH_REG = 0x05;
		const uint8_t MAX31856_LTHFTL_REG = 0x06;
		const uint8_t MAX31856_LTLFTH_REG = 0x07;
		const uint8_t MAX31856_LTLFTL_REG = 0x08;
		const uint8_t MAX31856_CJTO_REG = 0x09;
		const uint8_t MAX31856_CJTH_REG = 0x0A;
		const uint8_t MAX31856_CJTL_REG = 0x0B;
		const uint8_t MAX31856_LTCBH_REG = 0x0C;
		const uint8_t MAX31856_LTCBM_REG = 0x0D;
		const uint8_t MAX31856_LTCBL_REG = 0x0E;
		const uint8_t MAX31856_SR_REG = 0x0F;

		static const uint8_t MAX31856_FAULT_NONE = 0x00;
		static const uint8_t MAX31856_FAULT_CJRANGE = 0x80;
		static const uint8_t MAX31856_FAULT_TCRANGE = 0x40;
		static const uint8_t MAX31856_FAULT_CJHIGH = 0x20;
		static const uint8_t MAX31856_FAULT_CJLOW = 0x10;
		static const uint8_t MAX31856_FAULT_TCHIGH = 0x08;
		static const uint8_t MAX31856_FAULT_TCLOW = 0x04;
		static const uint8_t MAX31856_FAULT_OVUV = 0x02;
		static const uint8_t MAX31856_FAULT_OPEN = 0x01;
		static const uint8_t MAX31856_FAULT_ALL = 0xFF;

	private:
		void oneshotTemperature();
	};

} // namespace MAX31856