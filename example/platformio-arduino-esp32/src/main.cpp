#include "esp-max318-thermocouple/max31855.hxx"
#include "esp-max318-thermocouple/max31856.hxx"
#include "esp-max318-thermocouple/spimanager.hxx"
#include <Arduino.h>
#include <memory>

static const char *TAG = "MAX31856_Example";

using namespace ESP_MAX318_THERMOCOUPLE;

SPIManager *manager = nullptr;

std::shared_ptr<MAX31856> thermocouple1 = nullptr;
std::shared_ptr<MAX31855> thermocouple2 = nullptr;

// see the esp-idf example for a more complete example.

void setup() {
  Serial.begin(115200);

  Serial.printf("Starting MAX31856 Example Application (Compiled: %s %s)",
                __DATE__, __TIME__);

  // attach the manager to HSPI
  // SPIManager manager(HSPI_HOST);

  // or, change pins:
  auto spiconfig = SPIManager::defaultBusConfig;
  spiconfig.mosi_io_num = GPIO_NUM_13;
  spiconfig.miso_io_num = GPIO_NUM_12;
  spiconfig.sclk_io_num = GPIO_NUM_14;
  manager = new SPIManager(SPI2_HOST, spiconfig);

  // register a few MAX31856 devices, different CS pin for each device on the
  // same bus the esp32 supports only up to 3 devices on the same bus

  spi_device_interface_config_t device1SpiConfig =
      MAX31856::defaultSpiDeviceConfig;
  device1SpiConfig.spics_io_num = 27;

  MAX31856::MAX31856Config device1Config;

  thermocouple1 =
      manager->CreateDevice<MAX31856>(device1Config, device1SpiConfig);

  // the max31855 has less options than the 56, see the appropriate headers
  MAX31855::MAX31855Config device2Config; // defaults
  device2Config.averaging_samples = AveragingSamples::AVG_1;
  auto device2SpiConfig = MAX31855::defaultSpiDeviceConfig;
  device2SpiConfig.spics_io_num = 15;

  thermocouple2 =
      manager->CreateDevice<MAX31855>(device2Config, device2SpiConfig);
}

void loop() {

  // read the temperature

  Serial.printf("\n\r\n\r");

  Result result;
  thermocouple1->read(result);
  Serial.printf(
      "Device 1 Temp: %.2f °C, %.2f °F Cold Junction: %.2f °C, %.2f °F",
      result.thermocouple_c, result.thermocouple_f, result.coldjunction_c,
      result.coldjunction_f);
  Serial.printf("Faults: %u", result.fault_bits);
  for (const auto &fault : result.fault) {
    Serial.printf("Fault: %s", fault.c_str());
  }

  Serial.printf("\n\r");

  thermocouple2->read(result);
  Serial.printf(
      "Device 2 Temp: %.2f °C, %.2f °F Cold Junction: %.2f °C, %.2f °F",
      result.thermocouple_c, result.thermocouple_f, result.coldjunction_c,
      result.coldjunction_f);
  Serial.printf("Faults: %u", result.fault_bits);
  for (const auto &fault : result.fault) {
    Serial.printf("Fault: %s", fault.c_str());
  }

  Serial.printf("\n\r");

  delay(1000);
}