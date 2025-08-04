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
			MAX31856_TCTYPE_UNKNOWN = 0xFF // Unknown type
		};

		// Note: in one_shot mode, an OC Fault check happens every conversion. In auto convert mode, it happens every 16th conversion. Auto convert works well with a sample average of 16 this way
		enum class OCFaultConfig
		{
			NONE = 0x00,
			OC_01 = 0x01, // for a thermocouple network less than 5k ohms
			OC_10 = 0x02, // for a thermocouple network greater than 5k ohms and less than 40k ohms with a time constant less than 2ms
			OC_11 = 0x03  // for a thermocouple network greater than 5k ohms and less than 40k ohms with a time constant greater than 2ms
		};

		struct MAX31856Config : MAX318Config
		{
			float temp_fault_low;				// Low temperature fault threshold
			float temp_fault_high;				// High temperature fault threshold
			float cold_junction_fault_low;		// Low cold junction fault threshold
			float cold_junction_fault_high;		// High cold junction fault threshold
			AveragingSamples averaging_samples; // 1, 2, 4, 8, or 16 samples for averaging
			ThermocoupleType type;				// Thermocouple type
			uint8_t fault_mask;					// Fault mask for disabling some fault checks
			float cold_junction_offset;			// Cold junction offset in Celsius, within a range of -8°C to +7.9375°C
			bool auto_convert;					// Auto convert mode, else 1-shot mode
			OCFaultConfig oc_fault_type;
		};

		static constexpr spi_device_interface_config_t defaultSpiDeviceConfig = {

			.command_bits = 0, // Default command bits
			.address_bits = 0, // Default address bits
			.dummy_bits = 0,
			.mode = 1,							 // Mode 1 is required by the MAX31856
			.clock_source = SPI_CLK_SRC_DEFAULT, // Default clock source
			.duty_cycle_pos = 0,				 // default duty cycle
			.cs_ena_pretrans = 0,
			.cs_ena_posttrans = 0,
			.clock_speed_hz = (1000000),				// 1 Mhz
			.input_delay_ns = 0,						// No input delay (default)
			.sample_point = SPI_SAMPLING_POINT_PHASE_0, // Default sampling point
			.spics_io_num = 27,
			.flags = 0,
			.queue_size = 1,
			.pre_cb = nullptr,
			.post_cb = nullptr,
		};

		static constexpr MAX31856Config defaultConfig = {
			.temp_fault_low = -270.0f,
			.temp_fault_high = 1372.0f,
			.cold_junction_fault_low = -64.0f,
			.cold_junction_fault_high = 125.0f,
			.averaging_samples = AveragingSamples::AVG_16,
			.type = ThermocoupleType::MAX31856_TCTYPE_K,
			.fault_mask = 0xFF,
			.cold_junction_offset = 0.0f,
			.auto_convert = true,
			.oc_fault_type = OCFaultConfig::OC_01,
		};

		MAX31856(spi_device_interface_config_t aSpiDeviceConfig);

		bool configure(const MAX318Config *aConfig, const spi_device_handle_t &aHandle);

		ThermocoupleType getType(int &anOutError);

		virtual bool read(Result &anOutResult) override;

		struct CR0
		{
			const uint8_t readAddress = 0x00;
			const uint8_t writeAddress = 0x80;

			// all registers default to a value of 0
			struct Register
			{
				const uint8_t conversionMode = 0b01000000; // 1 to enable auto conversion, zero for one-shot mode.
				const uint8_t requestOneShot = 0b00100000; // 1 to request a one-shot conversion, self-clears to zero
				const uint8_t ocFault1 = 0b00010000;
				const uint8_t ocFault0 = 0b00001000;
				const uint8_t coldJunctionSensor = 0b00000100; // 0 = enabled, 1 = disabled
				const uint8_t faultMode = 0b00000010;		   // comparator = 0, interrupt = 1
				const uint8_t faultClear = 0b00000001;		   // Clear all faults
				const uint8_t filterHz = 0b00000000;		   // Filter frequency, 0 = 60Hz, 1 = 50Hz
			};
		};

		struct CR1
		{
			const uint8_t readAddress = 0x01;
			const uint8_t writeAddress = 0x81;

		const uint8_t CR1_TC_B = 0x00;
		const uint8_t CR1_TC_E = 0x01;
		const uint8_t CR1_TC_J = 0x02;
		const uint8_t CR1_TC_K = 0x03;
		const uint8_t CR1_TC_N = 0x04;
		const uint8_t CR1_TC_R = 0x05;
		const uint8_t CR1_TC_S = 0x06;
		const uint8_t CR1_TC_T = 0x07;
		const uint8_t CR1_VMODE_G8 = 0x08;
		const uint8_t CR1_VMODE_G32 = 0x0C;

		const uint8_t CR1_AVG_MODE_1 = 0x00;  // No averaging
		const uint8_t CR1_AVG_MODE_2 = 0x10;  // Enable + 2 samples
		const uint8_t CR1_AVG_MODE_4 = 0x30;  // Enable + 4 samples
		const uint8_t CR1_AVG_MODE_8 = 0x50;  // Enable + 8 samples
		const uint8_t CR1_AVG_MODE_16 = 0x70; // Enable + 16 samples

		const uint8_t CR1_REG = 0x01;
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
		bool setConvertType(bool isAutoConvert, int &anOutError);
		bool setTempFaultThreshholds(float aLow, float aHigh, int &anOutError);
		bool setColdJunctionFaultThreshholds(float aLow, float aHigh, int &anOutError);
		bool oneShotTemperature(int &anOutError);
		bool setType(ThermocoupleType aType, int &anOutError);
		bool setColdJunctionOffset(float anOffsetCelsius, int &anOutError);
		bool setFaultMask(uint8_t aMask, int &anOutError);
		uint8_t readFaultMask(int &anOutError);
		bool setAveragingMode(AveragingSamples aSamples, int &anOutError);

		MAX31856Config myConfig;
	};

} // namespace MAX31856