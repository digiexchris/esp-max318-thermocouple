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

	/**
	 * @brief MAX31855 SPI device class
	 *
	 * This class provides a simple interface to read temperature data from the MAX31855 thermocouple-to-digital converter.
	 * It uses SPI communication to interact with the device.
	 *
	 * It implements some features in software that are available in hardware in the MAX31856, such as:
	 * Over and under temperature detection
	 * Sample averaging
	 */
	class MAX31855 : public MAX318_Base
	{
	public:
		static constexpr spi_device_interface_config_t defaultSpiDeviceConfig = {
			.command_bits = 0, // Default command bits
			.address_bits = 0, // Default address bits
			.dummy_bits = 0,
			.mode = 0,									// mode 0 is required by the MAX31855
			.clock_source = SPI_CLK_SRC_DEFAULT,		// Default clock source
			.duty_cycle_pos = 0,						// Default duty cycle
			.cs_ena_pretrans = 0,						// No pre-transmission CS enable
			.cs_ena_posttrans = 0,						// No post-transmission CS enable
			.clock_speed_hz = (1000000),				// 1 Mhz
			.input_delay_ns = 0,						// No input delay (default)
			.sample_point = SPI_SAMPLING_POINT_PHASE_0, // Default sampling point
			.spics_io_num = 27,							// Manually Control CS
			.flags = SPI_DEVICE_HALFDUPLEX,
			.queue_size = 1,
			.pre_cb = nullptr,
			.post_cb = nullptr,
		};

		struct MAX31855Config : MAX318Config
		{
			float temp_fault_low;				// Low temperature fault threshold
			float temp_fault_high;				// High temperature fault threshold
			float cold_junction_fault_low;		// Low cold junction fault threshold
			float cold_junction_fault_high;		// High cold junction fault threshold
			AveragingSamples averaging_samples; // 1, 2, 4, 8, or 16 samples for averaging
		};

		static constexpr MAX31855Config defaultConfig = {
			.temp_fault_low = -270.0f,
			.temp_fault_high = 1372.0f,
			.cold_junction_fault_low = -64.0f,
			.cold_junction_fault_high = 125.0f,
			.averaging_samples = AveragingSamples::AVG_16};

		MAX31855(spi_device_interface_config_t aSpiDeviceConfig);

		bool configure(const MAX318Config *aConfig, const spi_device_handle_t &aHandle);

		bool read(Result &anOutResult) override;

	private:
		uint32_t readRegister32(int &anOutError);
		MAX31855Config myConfig;
	};

} // namespace MAX31855