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

	constexpr uint8_t MAX31855_CR0_REG = 0x00;
	constexpr uint8_t MAX31855_CR0_AUTOCONVERT = 0x80;
	constexpr uint8_t MAX31855_CR0_1SHOT = 0x40;
	constexpr uint8_t MAX31855_CR0_OCFAULT1 = 0x20;
	constexpr uint8_t MAX31855_CR0_OCFAULT0 = 0x10;
	constexpr uint8_t MAX31855_CR0_CJ = 0x08;
	constexpr uint8_t MAX31855_CR0_FAULT = 0x04;
	constexpr uint8_t MAX31855_CR0_FAULTCLR = 0x02;

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

	constexpr uint8_t MAX31855_FAULT_CJRANGE = 0x80;
	constexpr uint8_t MAX31855_FAULT_TCRANGE = 0x40;
	constexpr uint8_t MAX31855_FAULT_CJHIGH = 0x20;
	constexpr uint8_t MAX31855_FAULT_CJLOW = 0x10;
	constexpr uint8_t MAX31855_FAULT_TCHIGH = 0x08;
	constexpr uint8_t MAX31855_FAULT_TCLOW = 0x04;
	constexpr uint8_t MAX31855_FAULT_OVUV = 0x02;
	constexpr uint8_t MAX31855_FAULT_OPEN = 0x01;

	using MAX31855Callback = void (*)(Result *result);

	class MAX31855 : public MAX318_Base
	{
	public:
		MAX31855(gpio_num_t aCsPin, spi_host_device_t aHostId, const spi_device_interface_config_t &aDeviceConfig = defaultSpiDeviceConfig);

		uint8_t readFault(bool logFault = false);
		virtual void read(Result &anOutResult) override;

		static const spi_device_interface_config_t defaultSpiDeviceConfig;
	};

} // namespace MAX31855