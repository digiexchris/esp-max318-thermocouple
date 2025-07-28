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

    // std::shared_ptr<ESP_MAX318_THERMOCOUPLE::MAX31855> thermocouple2 = manager.CreateDevice<ESP_MAX318_THERMOCOUPLE::MAX31855>(GPIO_NUM_27);

    // auto thermocouple3 = manager.CreateDevice<ESP_MAX318_THERMOCOUPLE::MAX31856>(GPIO_NUM_19);

    // change the thermocouple type after creating if you want to
    // thermocouple1->setType(ESP_MAX318_THERMOCOUPLE::MAX31856::ThermocoupleType::MAX31856_TCTYPE_B);

    // set the fault thresholds for the devices
    // NOTE: this may not be working yet, I always see these faults set no matter what I do
    // thermocouple1->setTempFaultThreshholds(-40, 1000);
    // thermocouple3->setTempFaultThreshholds(0, 1370);
    // thermocouple1->setColdJunctionFaultThreshholds(-40, 140);
    // thermocouple3->setColdJunctionFaultThreshholds(0, 140);

    // disable some faults
    // thermocouple1->setFaultMask(ESP_MAX318_THERMOCOUPLE::MAX31856::MAX31856_FAULT_CJHIGH | ESP_MAX318_THERMOCOUPLE::MAX31856::MAX31856_FAULT_CJLOW);

    // or turn them all off
    // thermocouple1->setFaultMask(ESP_MAX318_THERMOCOUPLE::MAX31856::MAX31856_FAULT_ALL);

    // read the fault mask
    // uint8_t faultMask = thermocouple1->readFaultMask();
    // ESP_LOGI(TAG, "Fault Mask: 0x%02X", faultMask);

    while (42)
    {
        // read the temperature

        ESP_MAX318_THERMOCOUPLE::Result result;
        thermocouple1->read(result);
        // ESP_LOGI(TAG, "Temperature from device 1: %.2f °C", result.thermocouple_c);

        // and maybe the cold junction, both in F if you like
        // thermocouple2->read(result);
        // ESP_LOGI(TAG, "Temperature from device 2: %.2f °F, Cold Junction: %.2f °F", result.thermocouple_f, result.coldjunction_f);

        // and optionally the fault
        // thermocouple3->read(result);
        ESP_LOGI(TAG, "Temperature from device 3: %.2f °C, Cold Junction: %.2f °C, Fault: %u", result.thermocouple_c, result.coldjunction_c, result.fault_bits);

        for (const auto &fault : result.fault)
        {
            ESP_LOGI(TAG, "Fault: %s", fault.c_str());
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}