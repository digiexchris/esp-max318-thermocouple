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

namespace MAX31856
{

	constexpr uint8_t MAX31856_CR0_REG = 0x00;
	constexpr uint8_t MAX31856_CR0_AUTOCONVERT = 0x80;
	constexpr uint8_t MAX31856_CR0_1SHOT = 0x40;
	constexpr uint8_t MAX31856_CR0_OCFAULT1 = 0x20;
	constexpr uint8_t MAX31856_CR0_OCFAULT0 = 0x10;
	constexpr uint8_t MAX31856_CR0_CJ = 0x08;
	constexpr uint8_t MAX31856_CR0_FAULT = 0x04;
	constexpr uint8_t MAX31856_CR0_FAULTCLR = 0x02;

	constexpr uint8_t MAX31856_CR1_REG = 0x01;
	constexpr uint8_t MAX31856_MASK_REG = 0x02;
	constexpr uint8_t MAX31856_CJHF_REG = 0x03;
	constexpr uint8_t MAX31856_CJLF_REG = 0x04;
	constexpr uint8_t MAX31856_LTHFTH_REG = 0x05;
	constexpr uint8_t MAX31856_LTHFTL_REG = 0x06;
	constexpr uint8_t MAX31856_LTLFTH_REG = 0x07;
	constexpr uint8_t MAX31856_LTLFTL_REG = 0x08;
	constexpr uint8_t MAX31856_CJTO_REG = 0x09;
	constexpr uint8_t MAX31856_CJTH_REG = 0x0A;
	constexpr uint8_t MAX31856_CJTL_REG = 0x0B;
	constexpr uint8_t MAX31856_LTCBH_REG = 0x0C;
	constexpr uint8_t MAX31856_LTCBM_REG = 0x0D;
	constexpr uint8_t MAX31856_LTCBL_REG = 0x0E;
	constexpr uint8_t MAX31856_SR_REG = 0x0F;

	constexpr uint8_t MAX31856_FAULT_CJRANGE = 0x80;
	constexpr uint8_t MAX31856_FAULT_TCRANGE = 0x40;
	constexpr uint8_t MAX31856_FAULT_CJHIGH = 0x20;
	constexpr uint8_t MAX31856_FAULT_CJLOW = 0x10;
	constexpr uint8_t MAX31856_FAULT_TCHIGH = 0x08;
	constexpr uint8_t MAX31856_FAULT_TCLOW = 0x04;
	constexpr uint8_t MAX31856_FAULT_OVUV = 0x02;
	constexpr uint8_t MAX31856_FAULT_OPEN = 0x01;

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

	struct Result
	{
		float coldjunction_c;
		float coldjunction_f;
		float thermocouple_c;
		float thermocouple_f;
		uint8_t fault;
	};

	using MAX31856Callback = void (*)(Result *result);

	class MAX31856
	{
	public:
		MAX31856(spi_host_device_t aHostId, bool aConfigureBus = true, spi_bus_config_t aBusConfig = defaultSpi3BusConfig);

		// index is intentionally not defaulted here, so you don't accidentally overwrite a previous device
		void AddDevice(ThermocoupleType aType, gpio_num_t aCsPin, uint8_t anIndex);

		ThermocoupleType getType(uint8_t anIndex = 0);
		void setTempFaultThreshholds(float aLow, float aHigh, uint8_t anIndex = 0);
		void setColdJunctionFaultThreshholds(float aLow, float aHigh, uint8_t anIndex = 0);
		uint8_t readFault(bool logFault = false, uint8_t anIndex = 0);
		void read(Result &anOutResult, uint8_t anIndex = 0);
		void setType(ThermocoupleType aType, uint8_t anIndex = 0);
		void oneshotTemperature(uint8_t anIndex = 0);

	private:
		// since we're using a common frequency, bus config, and managing the CS pin ourselves, we can use a common handle
		spi_device_handle_t mySpiDeviceHandle;
		spi_host_device_t myHostId;
		gpio_num_t myCsPin[3];
		uint8_t myNumDevices = 0;

		void writeRegister(uint8_t anAddress, uint8_t someData, uint8_t anIndex = 0);
		uint8_t readRegister(uint8_t anAddress, uint8_t anIndex = 0);
		uint8_t readFastRegister(uint8_t anAddress, uint8_t anIndex = 0);
		uint16_t readRegister16(uint8_t anAddress, uint8_t anIndex = 0);
		uint32_t readRegister24(uint8_t anAddress, uint8_t anIndex = 0);

		static spi_bus_config_t defaultSpi3BusConfig;
	};

} // namespace MAX31856