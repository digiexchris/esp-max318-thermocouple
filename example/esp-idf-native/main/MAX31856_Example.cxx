#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <memory>
#include "esp-max318-thermocouple/max31856.hxx"
#include "esp-max318-thermocouple/max31855.hxx"
#include "esp-max318-thermocouple/spimanager.hxx"

static const char *TAG = "MAX31856_Example";

extern "C" void app_main()
{
    ESP_LOGI(TAG, "Starting MAX31856 Example Application (Compiled: %s %s)", __DATE__, __TIME__);

    // attach the manager to HSPI
    // ESP_MAX318_THERMOCOUPLE::SPIManager manager(HSPI_HOST);

    // or, change pins:
    auto spiconfig = ESP_MAX318_THERMOCOUPLE::SPIManager::defaultSpiBusConfig;
    spiconfig.mosi_io_num = GPIO_NUM_13;
    spiconfig.miso_io_num = GPIO_NUM_12;
    spiconfig.sclk_io_num = GPIO_NUM_14;
    ESP_MAX318_THERMOCOUPLE::SPIManager manager(SPI3_HOST, true, spiconfig);

    // register a few MAX31856 devices, different CS pin for each device on the same bus
    // the esp32 supports only up to 3 devices on the same bus

    std::shared_ptr<ESP_MAX318_THERMOCOUPLE::MAX31856> thermocouple1 = manager.CreateDevice<ESP_MAX318_THERMOCOUPLE::MAX31856>(GPIO_NUM_15);

    std::shared_ptr<ESP_MAX318_THERMOCOUPLE::MAX31856> thermocouple2 = manager.CreateDevice<ESP_MAX318_THERMOCOUPLE::MAX31856>(GPIO_NUM_5);

    std::shared_ptr<ESP_MAX318_THERMOCOUPLE::MAX31855> thermocouple3 = manager.CreateDevice<ESP_MAX318_THERMOCOUPLE::MAX31855>(GPIO_NUM_4);

    esp_err_t ret = ESP_OK;

    // change the thermocouple type after creating if you want to
    // thermocouple1->setType(ESP_MAX318_THERMOCOUPLE::MAX31856::ThermocoupleType::MAX31856_TCTYPE_B, ret);
    // assert(ret == ESP_OK); // assert if there was an error setting the type
    // thermocouple2->setType(ESP_MAX318_THERMOCOUPLE::MAX31856::ThermocoupleType::MAX31856_TCTYPE_K, ret);
    // setType also returns false if it did not succeed, if that's useful.

    // set the fault thresholds for the devices
    thermocouple1->setTempFaultThreshholds(-40, 1000, ret); // Supported by the device directly
    assert(ret == ESP_OK);
    thermocouple3->setTempFaultThreshholds(10, 1370, ret); // Emulated by this library
    assert(ret == ESP_OK);
    thermocouple1->setColdJunctionFaultThreshholds(-40, 140, ret);
    assert(ret == ESP_OK);
    thermocouple2->setColdJunctionFaultThreshholds(0, 140, ret);
    assert(ret == ESP_OK);

    // disable some faults
    // thermocouple1->setFaultMask(ESP_MAX318_THERMOCOUPLE::MAX31856::MAX31856_FAULT_CJHIGH | ESP_MAX318_THERMOCOUPLE::MAX31856::MAX31856_FAULT_CJLOW, ret);

    // or turn them all off
    // thermocouple1->setFaultMask(ESP_MAX318_THERMOCOUPLE::MAX31856::MAX31856_FAULT_ALL, ret);

    while (42)
    {
        // read the temperature

        ESP_LOGI(TAG, "\n\r\n\r");

        ESP_MAX318_THERMOCOUPLE::Result result;
        thermocouple1->read(result);
        ESP_LOGI(TAG, "Device 1 Temp: %.2f °C, %.2f °F Cold Junction: %.2f °C, %.2f °F", result.thermocouple_c, result.thermocouple_f, result.coldjunction_c, result.coldjunction_f);
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

        ESP_LOGI(TAG, "\n\r");

        thermocouple3->read(result);
        ESP_LOGI(TAG, "Device 3 Temp: %.2f °C, %.2f °F Cold Junction: %.2f °C, %.2f °F", result.thermocouple_c, result.thermocouple_f, result.coldjunction_c, result.coldjunction_f);
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

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}