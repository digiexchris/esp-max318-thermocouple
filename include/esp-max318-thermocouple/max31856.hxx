#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "soc/gpio_struct.h"

#include <esp_log.h>

#include "esp-max318-thermocouple/max318.hxx"

namespace ESP_MAX318_THERMOCOUPLE {

using MAX31856Callback = void (*)(Result *result);

class MAX31856 : public MAX318_Base {
public:
  enum class ThermocoupleType {
    TYPE_B = 0b0000,
    TYPE_E = 0b0001,
    TYPE_J = 0b0010,
    TYPE_K = 0b0011,
    TYPE_N = 0b0100,
    TYPE_R = 0b0101,
    TYPE_S = 0b0110,
    TYPE_T = 0b0111,
    TYPE_VMODE_G8 = 0b1000,
    TYPE_VMODE_G32 = 0b1100,
    TYPE_UNKNOWN = 0xFF // Unknown type
  };

  // Note: in one_shot mode, an OC Fault check happens every conversion. In auto
  // convert mode, it happens every 16th conversion. Auto convert works well
  // with a sample average of 16 this way
  enum class OCFaultConfig {
    NONE = 0x00,
    OC_01 = 0x01, // for a thermocouple network less than 5k ohms
    OC_10 = 0x02, // for a thermocouple network greater than 5k ohms and less
                  // than 40k ohms with a time constant less than 2ms
    OC_11 = 0x03  // for a thermocouple network greater than 5k ohms and less
                  // than 40k ohms with a time constant greater than 2ms
  };

  struct MAX31856Config : MAX318Config {
    float temp_fault_low = -270.0f;         // Low temperature fault threshold
    float temp_fault_high = 1372.0f;        // High temperature fault threshold
    float cold_junction_fault_low = -64.0f; // Low cold junction fault threshold
    float cold_junction_fault_high =
        125.0f; // High cold junction fault threshold
    AveragingSamples averaging_samples =
        AveragingSamples::AVG_1; // 1, 2, 4, 8, or 16 samples for averaging
    ThermocoupleType type = ThermocoupleType::TYPE_K; // Thermocouple type
    uint8_t fault_mask =
        FaultMask::faultNone; // Fault mask for disabling some fault checks
    float cold_junction_offset = 0.0f; // Cold junction offset in Celsius,
                                       // within a range of -8°C to +7.9375°C
    bool auto_convert = true;          // Auto convert mode, else 1-shot mode
    OCFaultConfig oc_fault_type = OCFaultConfig::OC_01; // Open circuit fault
  };

  static constexpr spi_device_interface_config_t defaultSpiDeviceConfig = {

      .command_bits = 0, // Default command bits
      .address_bits = 0, // Default address bits
      .dummy_bits = 0,
      .mode = 1,                           // Mode 1 is required by the MAX31856
      .clock_source = SPI_CLK_SRC_DEFAULT, // Default clock source
      .duty_cycle_pos = 0,                 // default duty cycle
      .cs_ena_pretrans = 0,
      .cs_ena_posttrans = 0,
      .clock_speed_hz = (1000000),                // 1 Mhz
      .input_delay_ns = 0,                        // No input delay (default)
      .sample_point = SPI_SAMPLING_POINT_PHASE_0, // Default sampling point
      .spics_io_num = 15,
      .flags = 0,
      .queue_size = 1,
      .pre_cb = nullptr,
      .post_cb = nullptr,
  };

  MAX31856(spi_device_interface_config_t aSpiDeviceConfig);

  bool configure(const MAX318Config *aConfig,
                 const spi_device_handle_t &aHandle);

  ThermocoupleType getType(int &anOutError);

  virtual bool read(Result &anOutResult) override;

  struct CR0 {
    static const uint8_t readAddress = 0x00;
    static const uint8_t writeAddress = 0x80;

    // all registers default to a value of 0
    // Register enabled bits:
    static const uint8_t autoConversionMode =
        0b10000000; // 1 to enable auto conversion, zero for one-shot mode.
    static const uint8_t requestOneShot =
        0b01000000; // 1 to request a one-shot conversion, self-clears to zero
    static const uint8_t ocFault1 = 0b00100000;
    static const uint8_t ocFault0 = 0b00010000;
    static const uint8_t coldJunctionSensor =
        0b00001000; // 0 = enabled, 1 = disabled
    static const uint8_t faultMode =
        0b00000100; // comparator = 0, interrupt = 1
    static const uint8_t faultClear = 0b00000010; // Clear all faults
    static const uint8_t filterHz =
        0b00000001; // Filter frequency, 0 = 60Hz, 1 = 50Hz
  };

  struct CR1 {
    static const uint8_t readAddress = 0x01;
    static const uint8_t writeAddress = 0x81;
    static const uint8_t thermocoupleTypBits =
        0b00001111; // Thermocouple type registers
    static const uint8_t averagingModeBits =
        0b01110000; // Averaging mode registers

    // Register enabled bits:
    static const uint8_t thermocoupleTypeB = 0b00000000;
    static const uint8_t thermocoupleTypeE = 0b00000001;
    static const uint8_t thermocoupleTypeJ = 0b00000010;
    static const uint8_t thermocoupleTypeK = 0b00000011;
    static const uint8_t thermocoupleTypeN = 0b00000100;
    static const uint8_t thermocoupleTypeR = 0b00000101;
    static const uint8_t thermocoupleTypeS = 0b00000110;
    static const uint8_t thermocoupleTypeT = 0b00000111;
    static const uint8_t thermocoupleTypeVmodeG8 = 0b00001000;
    static const uint8_t thermocoupleTypeVmodeG32 = 0b00001100;

    static const uint8_t averagingMode1 = 0b00000000;
    static const uint8_t averagingMode2 = 0b00010000;
    static const uint8_t averagingMode4 = 0b00110000;
    static const uint8_t averagingMode8 = 0b01010000;
    static const uint8_t averagingMode16 = 0b01110000;
  };

  struct FaultMask {
    static const uint8_t readAddress = 0x02;
    static const uint8_t writeAddress = 0x82;
    static const uint8_t faultMaskBits = 0b00011111;
    static const uint8_t reservedBits =
        0b11100000; // Reserved bits, should be read and written as read

    // Register enabled bits:
    static const uint8_t faultNone = 0b00000000;   // No faults disabled
    static const uint8_t faultAll = 0b00111111;    // All faults disabled
    static const uint8_t faultCJHigh = 0b00100000; // Cold junction high fault
    static const uint8_t faultCJLow = 0b00010000;  // Cold junction low fault
    static const uint8_t faultTCHigh = 0b00001000; // Thermocouple high fault
    static const uint8_t faultTCLow = 0b00000100;  // Thermocouple low fault
    static const uint8_t faultOVUV =
        0b00000010; // Overvoltage/undervoltage fault
    static const uint8_t faultOpenCircuit = 0b00000001; // Open circuit fault
  };

  struct CJHF {
    static const uint8_t readAddress = 0x03;
    static const uint8_t writeAddress = 0x83;
    static const uint8_t signBit = 0b10000000;
    static const uint8_t valueBits = 0b01111111;
  };

  struct CJLF {
    static const uint8_t readAddress = 0x04;
    static const uint8_t writeAddress = 0x84;
    static const uint8_t signBit = 0b10000000;
    static const uint8_t valueBits = 0b01111111;
  };

  struct LTHFT {
    struct MSB {
      static const uint8_t readAddress = 0x05;
      static const uint8_t writeAddress = 0x85;
      static const uint8_t signBit = 0b10000000;
      static const uint8_t valueBits = 0b01111111;
    };
    struct LSB {
      static const uint8_t readAddress = 0x06;
      static const uint8_t writeAddress = 0x86;
      static const uint8_t valueBits = 0b11111111;
    };
  };

  struct LTLFT {
    struct MSB {
      static const uint8_t readAddress = 0x07;
      static const uint8_t writeAddress = 0x87;
      static const uint8_t signBit = 0b10000000;
      static const uint8_t valueBits = 0b01111111;
    };
    struct LSB {
      static const uint8_t readAddress = 0x08;
      static const uint8_t writeAddress = 0x88;
      static const uint8_t valueBits = 0b11111111;
    };
  };

  struct CJTO {
    static const uint8_t readAddress = 0x09;
    static const uint8_t writeAddress = 0x89;
    static const uint8_t signBit = 0b10000000;
    static const uint8_t valueBits = 0b01111111;
  };

  struct CJT {
    struct MSB {
      static const uint8_t readAddress = 0x0A;
      static const uint8_t writeAddress = 0x8A;
      static const uint8_t signBit = 0b10000000;
      static const uint8_t valueBits = 0b01111111;
    };
    struct LSB {
      static const uint8_t readAddress = 0x0B;
      static const uint8_t writeAddress = 0x8B;
      static const uint8_t valueBits = 0b11111111;
    };
  };

  struct LTC {
    struct Byte2 {
      static const uint8_t readAddress = 0x0C;
      static const uint8_t signBit = 0b10000000;
      static const uint8_t valueBits = 0b01111111;
    };
    struct Byte1 {
      static const uint8_t readAddress = 0x0D;
      static const uint8_t valueBits = 0b11111111;
    };
    struct Byte0 {
      static const uint8_t readAddress = 0x0E;
      static const uint8_t valueBits = 0b11111111;
    };
  };

  struct SR {
    static const uint8_t readAddress = 0x0F;

    // Register enabled bits:
    static const uint8_t cjRange = 0b10000000;
    static const uint8_t tcRange = 0b01000000;
    static const uint8_t cjHigh = 0b00100000;
    static const uint8_t cjLow = 0b00010000;
    static const uint8_t tcHigh = 0b00001000;
    static const uint8_t tcLow = 0b00000100;
    static const uint8_t ovuv = 0b00000010;
    static const uint8_t open = 0b00000001;
  };

private:
  void setConvertType(bool isAutoConvert, int &anOutError);
  void setTempFaultThreshholds(float aLow, float aHigh, int &anOutError);
  void setColdJunctionFaultThreshholds(float aLow, float aHigh,
                                       int &anOutError);
  void oneShotTemperature(int &anOutError);
  void setType(ThermocoupleType aType, int &anOutError);
  void setColdJunctionOffset(float anOffsetCelsius, int &anOutError);
  void enableFault(uint8_t aMask, int &anOutError);
  void disableFault(uint8_t aMask, int &anOutError);
  void setFaultMask(uint8_t aMask, int &anOutError);
  uint8_t readFaultMask(int &anOutError);
  void setAveragingMode(AveragingSamples aSamples, int &anOutError);

  float readCJTemp(int &anOutError);
  float readLTC(int &anOutError);
  uint8_t readFault(int &anOutError);

  // used in one-shot-mode only. Faults auto de-assert in autoconvert mode.
  void clearFault(int &anOutError);

  MAX31856Config myConfig;
};

} // namespace ESP_MAX318_THERMOCOUPLE