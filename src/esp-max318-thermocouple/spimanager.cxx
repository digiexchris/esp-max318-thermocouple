#include "esp-max318-thermocouple/spimanager.hxx"
#include "esp-max318-thermocouple/max318.hxx"
#include "esp_log.h"

namespace ESP_MAX318_THERMOCOUPLE
{

    const char *SPIManagerTAG = "SPIManager";

    SPIManager::SPIManager(spi_host_device_t aHostId, bool aConfigureBus, spi_bus_config_t aBusConfig) : myHostId(aHostId)
    {
        ESP_LOGI(SPIManagerTAG, "Initialize SPIManager");
        esp_err_t ret;

        if (aConfigureBus)
        {
            // if it's already initalized, that's cool. We can add devices to it later
            ret = spi_bus_initialize(aHostId, &aBusConfig, 0);
            if (ret != ESP_ERR_INVALID_STATE && ret != ESP_OK)
            {
                ESP_ERROR_CHECK(ret);
            }
        }
    }

    spi_bus_config_t SPIManager::defaultSpi3BusConfig = {
        .mosi_io_num = GPIO_NUM_32,
        .miso_io_num = GPIO_NUM_39,
        .sclk_io_num = GPIO_NUM_25,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
} // namespace ESP_MAX318_THERMOCOUPLE