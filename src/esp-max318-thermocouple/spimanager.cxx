#include "esp-max318-thermocouple/spimanager.hxx"
#include "esp-max318-thermocouple/max318.hxx"
#include "esp_log.h"

namespace ESP_MAX318_THERMOCOUPLE
{

    const char *SPIManagerTAG = "SPIManager";

    SPIManager::SPIManager(spi_host_device_t aHostId, spi_bus_config_t aBusConfig, bool aConfigureBus) : myHostId(aHostId)
    {
        ESP_LOGI(SPIManagerTAG, "Initialize SPIManager");
        esp_err_t ret;

        if (aConfigureBus)
        {
            // if it's already initalized, that's cool. We can add devices to it later
            ret = spi_bus_initialize(aHostId, &aBusConfig, SPI_DMA_CH_AUTO);
            if (ret != ESP_ERR_INVALID_STATE && ret != ESP_OK)
            {
                ESP_ERROR_CHECK(ret);
            }
        }
    }

} // namespace ESP_MAX318_THERMOCOUPLE