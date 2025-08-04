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
        SPIManager(spi_host_device_t aHostId, spi_bus_config_t aBusConfig = {}, bool aConfigureBus = true);

        template <typename Derived>
        std::shared_ptr<Derived> CreateDevice(const MAX318Config &aConfig, const spi_device_interface_config_t &anSpiDeviceConfig)
        {
            if (myNumDevices == 3)
            {
                ESP_LOGE("SPIManager", "An SPI bus can only have at most 3 devices on the ESP32");
                assert(false);
            }

            spi_device_handle_t handle;
            esp_err_t ret = spi_bus_add_device(myHostId, &anSpiDeviceConfig, &handle);
            assert(ret == ESP_OK);

            std::shared_ptr<Derived> newDevice = std::make_shared<Derived>(anSpiDeviceConfig);

            newDevice->configure(&aConfig, handle);

            this->myDevices[myNumDevices] = newDevice;
            this->myNumDevices++;

            return newDevice;
        }

        static constexpr spi_bus_config_t defaultBusConfig = {
            .mosi_io_num = GPIO_NUM_13,
            .miso_io_num = GPIO_NUM_12,
            .sclk_io_num = GPIO_NUM_14,
            .quadwp_io_num = -1,   // Not used
            .quadhd_io_num = -1,   // Not used
            .max_transfer_sz = 32, // Maximum bytes per transaction
            .flags = SPICOMMON_BUSFLAG_MASTER,
        };

    private:
        spi_host_device_t myHostId;
        std::shared_ptr<MAX318_Base> myDevices[3];
        uint8_t myNumDevices = 0;
    };
}
