#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "max31856.hxx"

static const char *TAG = "MAX31856_Example";

extern "C" void app_main()
{
    ESP_LOGI(TAG, "Starting MAX31856 Example Application");

    // attach the driver to SPI3
    MAX31856::MAX31856 thermocouple(SPI3_HOST);

    // register a few MAX31856 devices, different CS pin for each device on the same bus
    // the esp32 supports only up to 3 devices on the same bus
    thermocouple.AddDevice(MAX31856::ThermocoupleType::MAX31856_TCTYPE_K, GPIO_NUM_5, 0);
    thermocouple.AddDevice(MAX31856::ThermocoupleType::MAX31856_TCTYPE_J, GPIO_NUM_18, 1);
    thermocouple.AddDevice(MAX31856::ThermocoupleType::MAX31856_TCTYPE_N, GPIO_NUM_19, 2);

    // change the thermocouple type after adding if you want to
    // thermocouple.setType(MAX31856::ThermocoupleType::MAX31856_TCTYPE_B, 0);

    // set the fault thresholds for the devices
    // NOTE: this may not be working yet, I always see these faults set no matter what I do
    //  thermocouple.setTempFaultThreshholds(-40, 140, 0);
    //  thermocouple.setTempFaultThreshholds(-40, 140, 1);
    //  thermocouple.setTempFaultThreshholds(-40, 140, 2);
    //  thermocouple.setColdJunctionFaultThreshholds(-40, 140, 0);
    //  thermocouple.setColdJunctionFaultThreshholds(-40, 140, 1);
    //  thermocouple.setColdJunctionFaultThreshholds(-40, 140, 2);

    while (42)
    {
        // read the temperature from each device
        for (int i = 0; i < 3; i++)
        {
            float temperature = thermocouple.readTemperature(i);
            ESP_LOGI(TAG, "Temperature from device %d: %.2f °C", i, temperature);
        }

        // read the cold junction temperature from each device
        for (int i = 0; i < 3; i++)
        {
            float cold_junction = thermocouple.readColdJunction(i);
            ESP_LOGI(TAG, "Cold Junction from device %d: %.2f °C", i, cold_junction);
        }

        // read the fault status from each device
        // NOTE: this may not be working yet, I always see these faults set no matter what I do
        //  for (int i = 0; i < 3; i++) {
        //      uint8_t fault = thermocouple.readFault(true, i);
        //      ESP_LOGI(TAG, "Fault status from device %d: 0x%02X", i, fault);
        //  }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}