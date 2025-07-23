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
    ESP_LOGI(TAG, "Starting MAX31856 Example Application");

    // attach the manager to SPI3
    ESP_MAX318_THERMOCOUPLE::SPIManager manager(SPI3_HOST);

    // register a few MAX31856 devices, different CS pin for each device on the same bus
    // the esp32 supports only up to 3 devices on the same bus

    std::shared_ptr<ESP_MAX318_THERMOCOUPLE::MAX31856> thermocouple1 = manager.CreateDevice<ESP_MAX318_THERMOCOUPLE::MAX31856>(GPIO_NUM_5);

    std::shared_ptr<ESP_MAX318_THERMOCOUPLE::MAX31855> thermocouple2 = manager.CreateDevice<ESP_MAX318_THERMOCOUPLE::MAX31855>(GPIO_NUM_18);

    auto thermocouple3 = manager.CreateDevice<ESP_MAX318_THERMOCOUPLE::MAX31856>(GPIO_NUM_19);

    // change the thermocouple type after creating if you want to
    thermocouple1->setType(ESP_MAX318_THERMOCOUPLE::MAX31856::ThermocoupleType::MAX31856_TCTYPE_B);

    // set the fault thresholds for the devices
    // NOTE: this may not be working yet, I always see these faults set no matter what I do
    thermocouple1->setTempFaultThreshholds(-40, 140);
    thermocouple3->setTempFaultThreshholds(0, 1370);
    thermocouple1->setColdJunctionFaultThreshholds(-40, 140);
    thermocouple3->setColdJunctionFaultThreshholds(0, 140);

    while (42)
    {
        // read the temperature

        ESP_MAX318_THERMOCOUPLE::Result result;
        thermocouple1->read(result);
        ESP_LOGI(TAG, "Temperature from device 1: %.2f °C", result.thermocouple_c);

        // and maybe the cold junction, both in F if you like
        thermocouple2->read(result);
        ESP_LOGI(TAG, "Temperature from device 2: %.2f °F, Cold Junction: %.2f °F", result.thermocouple_f, result.coldjunction_f);

        // and optionally the fault
        thermocouple3->read(result);
        ESP_LOGI(TAG, "Temperature from device 3: %.2f °F, Cold Junction: %.2f °F, Fault: %s", result.thermocouple_f, result.coldjunction_f, result.fault.c_str());

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}