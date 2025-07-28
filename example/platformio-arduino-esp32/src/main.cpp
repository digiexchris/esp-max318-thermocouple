#include <Arduino.h>
#include <memory>
#include "esp-max318-thermocouple/max31856.hxx"
#include "esp-max318-thermocouple/max31855.hxx"
#include "esp-max318-thermocouple/spimanager.hxx"

static const char *TAG = "MAX31856_Example";

ESP_MAX318_THERMOCOUPLE::SPIManager *manager = nullptr;

std::shared_ptr<ESP_MAX318_THERMOCOUPLE::MAX31856> thermocouple1 = nullptr;
std::shared_ptr<ESP_MAX318_THERMOCOUPLE::MAX31856> thermocouple2 = nullptr;
std::shared_ptr<ESP_MAX318_THERMOCOUPLE::MAX31855> thermocouple3 = nullptr;

void setup()
{
    Serial.begin(115200);

    Serial.printf("Starting MAX31856 Example Application (Compiled: %s %s)", __DATE__, __TIME__);

    // attach the manager to HSPI
    // ESP_MAX318_THERMOCOUPLE::SPIManager manager(HSPI_HOST);

    // or, change pins:
    auto spiconfig = ESP_MAX318_THERMOCOUPLE::SPIManager::defaultSpiBusConfig;
    spiconfig.mosi_io_num = GPIO_NUM_13;
    spiconfig.miso_io_num = GPIO_NUM_12;
    spiconfig.sclk_io_num = GPIO_NUM_14;
    manager = new ESP_MAX318_THERMOCOUPLE::SPIManager(SPI3_HOST, true, spiconfig);

    // register a few MAX31856 devices, different CS pin for each device on the same bus
    // the esp32 supports only up to 3 devices on the same bus

    thermocouple1 = manager->CreateDevice<ESP_MAX318_THERMOCOUPLE::MAX31856>(GPIO_NUM_15);

    thermocouple2 = manager->CreateDevice<ESP_MAX318_THERMOCOUPLE::MAX31856>(GPIO_NUM_5);

    thermocouple3 = manager->CreateDevice<ESP_MAX318_THERMOCOUPLE::MAX31855>(GPIO_NUM_4);

    // change the thermocouple type after creating if you want to
    // thermocouple1->setType(ESP_MAX318_THERMOCOUPLE::MAX31856::ThermocoupleType::MAX31856_TCTYPE_B);
    // thermocouple2->setType(ESP_MAX318_THERMOCOUPLE::MAX31856::ThermocoupleType::MAX31856_TCTYPE_K);

    int ret = 0; //this ends up being an esp_err_t error code
    // set the fault thresholds for the devices
    thermocouple1->setTempFaultThreshholds(-40, 1000, ret); // Supported by the device directly
    assert(ret == 0);

    thermocouple3->setTempFaultThreshholds(10, 1370, ret);  // Emulated by this library
    assert(ret == 0);
    thermocouple1->setColdJunctionFaultThreshholds(-40, 140, ret);
    assert(ret == 0);
    thermocouple2->setColdJunctionFaultThreshholds(0, 140, ret);
    assert(ret == ESP_OK); //should be usable, see esp_err_t for the codes.

    // disable some faults
    // thermocouple1->setFaultMask(ESP_MAX318_THERMOCOUPLE::MAX31856::MAX31856_FAULT_CJHIGH | ESP_MAX318_THERMOCOUPLE::MAX31856::MAX31856_FAULT_CJLOW);

    // or turn them all off
    // thermocouple1->setFaultMask(ESP_MAX318_THERMOCOUPLE::MAX31856::MAX31856_FAULT_ALL);
}

void loop()
{

    // read the temperature

    Serial.printf("\n\r\n\r");

    ESP_MAX318_THERMOCOUPLE::Result result;
    thermocouple1->read(result);
    Serial.printf("Device 1 Temp: %.2f °C, %.2f °F Cold Junction: %.2f °C, %.2f °F", result.thermocouple_c, result.thermocouple_f, result.coldjunction_c, result.coldjunction_f);
    Serial.printf("Faults: %u", result.fault_bits);
    for (const auto &fault : result.fault)
    {
        Serial.printf("Fault: %s", fault.c_str());
    }

    Serial.printf("\n\r");

    thermocouple2->read(result);
    Serial.printf("Device 2 Temp: %.2f °C, %.2f °F Cold Junction: %.2f °C, %.2f °F", result.thermocouple_c, result.thermocouple_f, result.coldjunction_c, result.coldjunction_f);
    Serial.printf("Faults: %u", result.fault_bits);
    for (const auto &fault : result.fault)
    {
        Serial.printf("Fault: %s", fault.c_str());
    }

    Serial.printf("\n\r");

    thermocouple3->read(result);
    Serial.printf("Device 3 Temp: %.2f °C, %.2f °F Cold Junction: %.2f °C, %.2f °F", result.thermocouple_c, result.thermocouple_f, result.coldjunction_c, result.coldjunction_f);
    Serial.printf("Faults: %u", result.fault_bits);
    for (const auto &fault : result.fault)
    {
        Serial.printf("Fault: %s", fault.c_str());
    }
    Serial.printf( "SPI Transaction probably succeeded %s\n\r", result.spi_success ? "true" : "false");
    if (!result.spi_success)
    {
        Serial.printf("Error Code: %d", result.error_code);
    }

    delay(1000); // wait for a second before the next read
}