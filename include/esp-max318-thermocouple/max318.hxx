#pragma once
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "soc/gpio_struct.h"
#include <string>

namespace ESP_MAX318_THERMOCOUPLE
{
    struct Result
    {
        float coldjunction_c;
        float coldjunction_f;
        float thermocouple_c;
        float thermocouple_f;
        std::string fault;
    };

    class MAX318_Base
    {
    public:
        MAX318_Base(gpio_num_t aCsPin, spi_host_device_t aHostId, const spi_device_interface_config_t &aDeviceConfig);

        virtual void read(Result &anOutResult) = 0;

        // generic SPI functions for any of this device class
        void writeRegister(uint8_t anAddress, uint8_t someData);
        uint8_t readRegister(uint8_t anAddress);
        uint8_t readFastRegister(uint8_t anAddress);
        uint16_t readRegister16(uint8_t anAddress);
        uint32_t readRegister24(uint8_t anAddress);

    protected:
        gpio_num_t myCsPin;
        spi_device_interface_config_t devcfg;
        spi_host_device_t myHostId;
        spi_device_handle_t mySpiDeviceHandle;
    };

    enum class MAX318Type
    {
        MAX31855,
        MAX31856,
    };

}