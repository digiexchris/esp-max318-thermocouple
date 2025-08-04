#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <memory>
#include "esp-max318-thermocouple/max31856.hxx"
#include "esp-max318-thermocouple/max31855.hxx"
#include "esp-max318-thermocouple/spimanager.hxx"

static const char *TAG = "MAX31856_Example";

using namespace ESP_MAX318_THERMOCOUPLE;

extern "C" void app_main()
{
    ESP_LOGI(TAG, "Starting MAX31856 Example Application (Compiled: %s %s)", __DATE__, __TIME__);

    // attach the manager to HSPI
    // ESP_MAX318_THERMOCOUPLE::SPIManager manager(HSPI_HOST);

    // or, change pins:
    spi_bus_config_t spiconfig = SPIManager::defaultBusConfig;
    spiconfig.mosi_io_num = GPIO_NUM_13;
    spiconfig.miso_io_num = GPIO_NUM_12;
    spiconfig.sclk_io_num = GPIO_NUM_14;
    SPIManager manager(SPI2_HOST, spiconfig);
    // alternatively you can configure the bus yourself, pass an empty config, and set aConfigureBus to false. This can be useful if another library is stealing the bus initialization away from you. Just be aware in this case that the 3 device per bus limit applies but this SPI manager doesn't know about other devices you add through other means.
    // SPIManager manager(SPI3_HOST, {}, false);

    //
    // create some device configurations (NOTE the esp32 supports only up to 3 devices on the same bus)

    // using the defaults specific to the 56, first one defaulting to pin 27
    // since we're using all defaults, we don't even need to make a config. CreateDevice will do it for us.
    // MAX31856::MAX31856Config device1Config = MAX31856::defaultConfig;

    // change the second one to use pin 15
    spi_device_interface_config_t device2SpiConfig = MAX31856::defaultSpiDeviceConfig;
    device2SpiConfig.spics_io_num = 27;
    MAX31856::MAX31856Config device2Config = MAX31856::defaultConfig;
    // can also set the specific device config all by hand
    device2Config = {
        .temp_fault_low = -270.0f,
        .temp_fault_high = 1372.0f,
        .cold_junction_fault_low = -12.0f,
        .cold_junction_fault_high = 100.0f,
        .averaging_samples = AveragingSamples::AVG_16,
        .type = MAX31856::ThermocoupleType::MAX31856_TCTYPE_K,
        .fault_mask = MAX31856::MAX31856_FAULT_ALL, // MAX31856::MAX31856_FAULT_CJHIGH | MAX31856::MAX31856_FAULT_CJLOW, // keep the all on escept for the CJ high and low faults
        .cold_junction_offset = 0.0f,
        .auto_convert = false,
        .oc_fault_type = MAX31856::OCFaultConfig::OC_01};

    // the max31855 has less options than the 56, see the appropriate headers
    auto device3Config = MAX31855::defaultConfig;
    device3Config.averaging_samples = AveragingSamples::AVG_1;
    auto device3SpiConfig = MAX31855::defaultSpiDeviceConfig;
    device3SpiConfig.spics_io_num = 15;

    // finally, create them!

    // create a completely default MAX31856 with default pins (27)
    // std::shared_ptr<MAX31856> thermocouple1 = manager.CreateDevice<MAX31856>(MAX31856::defaultConfig, MAX31856::defaultSpiDeviceConfig);

    // // customize both the device config and the spi config
    std::shared_ptr<MAX31856> thermocouple2 = manager.CreateDevice<MAX31856>(device2Config, device2SpiConfig);

    // customizing only the device config and using default spi device config
    // std::shared_ptr<MAX31855> thermocouple3 = manager.CreateDevice<MAX31855>(device3Config, device3SpiConfig);

    while (42)
    {
        // read the temperature

        // ESP_LOGI(TAG, "\n\r\n\r");

        ESP_MAX318_THERMOCOUPLE::Result result;
        // thermocouple1->read(result);
        // ESP_LOGI(TAG, "Device 1 Temp: %.2f °C, %.2f °F Cold Junction: %.2f °C, %.2f °F", result.thermocouple_c, result.thermocouple_f, result.coldjunction_c, result.coldjunction_f);
        // ESP_LOGI(TAG, "Faults: %u", result.fault_bits);
        // for (const auto &fault : result.fault)
        // {
        //     ESP_LOGI(TAG, "Fault: %s", fault.c_str());
        // }
        // ESP_LOGI(TAG, "SPI Transaction probably succeeded %s\n\r", result.spi_success ? "true" : "false");
        // if (!result.spi_success)
        // {
        //     ESP_LOGE(TAG, "Error Code: %d", result.error_code);
        // }

        ESP_LOGI(TAG, "\n\r");

        thermocouple2->read(result);
        ESP_LOGI(TAG, "Device 2 Temp: %.2f °C, %.2f °F Cold Junction: %.2f °C, %.2f °F", result.thermocouple_c, result.thermocouple_f, result.coldjunction_c, result.coldjunction_f);
        ESP_LOGI(TAG, "Faults: %u", result.fault_bits);
        for (const auto &fault : result.fault)
        {
            ESP_LOGI(TAG, "Fault: %s", fault.c_str());
        }
        ESP_LOGI(TAG, "SPI Transaction probably succeeded %s\n\r", result.spi_success ? "true" : "false");
        if (!result.spi_success)
        {
            ESP_LOGE(TAG, "Error Code: %d", result.error_code);
        }

        // ESP_LOGI(TAG, "\n\r");

        // thermocouple3->read(result);
        // ESP_LOGI(TAG, "Device 3 Temp: %.2f °C, %.2f °F Cold Junction: %.2f °C, %.2f °F", result.thermocouple_c, result.thermocouple_f, result.coldjunction_c, result.coldjunction_f);
        // ESP_LOGI(TAG, "Faults: %u", result.fault_bits);
        // for (const auto &fault : result.fault)
        // {
        //     ESP_LOGI(TAG, "Fault: %s", fault.c_str());
        // }
        // ESP_LOGI(TAG, "SPI Transaction probably succeeded %s\n\r", result.spi_success ? "true" : "false");
        // if (!result.spi_success)
        // {
        //     ESP_LOGE(TAG, "Error Code: %d", result.error_code);
        // }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}