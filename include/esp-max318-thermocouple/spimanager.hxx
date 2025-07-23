#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "esp_log.h"
#include "soc/gpio_struct.h"
#include <memory>
#include <cassert>

#include "esp-max318-thermocouple/max318.hxx"

namespace ESP_MAX318_THERMOCOUPLE
{
    class SPIManager
    {
    public:
        SPIManager(spi_host_device_t aHostId, bool aConfigureBus = true, spi_bus_config_t aBusConfig = defaultSpi3BusConfig);

        /*
        @brief Create a device of the specified type on the SPI bus.
        @param Derived The type of MAX318 device to create.
        @param aCsPin The GPIO pin number for the Chip Select (CS) line.
        @return A shared pointer to the created device.
        @note The function will assert if more than 3 devices are already created on the bus

        Example usage:

        auto thermocouple1 = manager.CreateDevice<MAX31856>(GPIO_NUM_5);
        auto thermocouple2 = manager.CreateDevice<MAX31855>(GPIO_NUM_18);
        std::shared_ptr<MAX31856> thermocouple3 = manager.CreateDevice<MAX31856>(GPIO_NUM_19);
        */
        template <typename Derived>
        std::shared_ptr<Derived> CreateDevice(gpio_num_t aCsPin)
        {
            if (myNumDevices == 3)
            {
                ESP_LOGE("SPIManager", "An SPI bus can only have at most 3 devices on the ESP32");
                assert(false);
            }

            std::shared_ptr<Derived> newDevice = std::make_shared<Derived>(aCsPin, myHostId);

            this->myDevices[myNumDevices] = newDevice;
            this->myNumDevices++;

            return newDevice;
        }

    private:
        // since we're using a common frequency, bus config, and managing the CS pin ourselves, we can use a common handle
        spi_host_device_t myHostId;
        std::shared_ptr<MAX318_Base> myDevices[3];
        uint8_t myNumDevices = 0;

        static spi_bus_config_t defaultSpi3BusConfig;
    };
}
