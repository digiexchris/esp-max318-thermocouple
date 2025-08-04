#include "sdkconfig.h"
#include <memory>
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

#include "esp-max318-thermocouple/max31856.hxx"

namespace ESP_MAX318_THERMOCOUPLE {

const char *TAG = "MAX31856";

MAX31856::MAX31856(spi_device_interface_config_t aSpiDeviceConfig)
    : MAX318_Base(aSpiDeviceConfig) {
  assert(aSpiDeviceConfig.mode == 1); // Mode 1 is required by the MAX31856
}

bool MAX31856::configure(const MAX318Config *aConfig,
                         const spi_device_handle_t &aHandle) {
  myConfig = *static_cast<const MAX31856Config *>(aConfig);
  mySpiDeviceHandle = aHandle;

  int error = 0;

  // // Set the MASK to 0xFF to disable all fault detection to test spi
  // communication
  disableFault(FaultMask::faultAll, error);
  assert(error == 0);

  // // Validate device communication by reading a known register
  uint8_t mask_reg = readFaultMask(error);
  assert(error == 0);
  if (mask_reg & FaultMask::faultAll) {
    ESP_LOGW(
        TAG,
        "Warning: setup returned unexpected values %02X, check SPI connection",
        mask_reg);
  }

  // turn them all back on again
  enableFault(FaultMask::faultAll, error);
  assert(error == 0);

  // Clear any existing faults since the default conversion mode is one-shot
  clearFault(error);
  assert(error == 0);

  // must be in one-shot mode to set the averaging mode
  setAveragingMode(myConfig.averaging_samples, error);
  assert(error == 0);

  if (myConfig.auto_convert) {
    // If auto convert mode is enabled, set the device to auto convert
    setConvertType(true, error);
    assert(error == 0);
  }

  // Ensure internal CJ temp sensor is enabled
  uint8_t cr0_val = readRegister(CR0::coldJunctionSensor, error);
  assert(error == 0);
  cr0_val &= ~CR0::coldJunctionSensor; // Clear CJ bit = use internal CJ
  writeRegister(CR0::coldJunctionSensor, cr0_val, error);
  assert(error == 0);

  // set default temp thresholds for the tc type
  setTempFaultThreshholds(myConfig.temp_fault_low, myConfig.temp_fault_high,
                          error);
  setColdJunctionFaultThreshholds(myConfig.cold_junction_fault_low,
                                  myConfig.cold_junction_fault_high, error);

  return true;
}

void MAX31856::enableFault(uint8_t aMask, int &anOutError) {
  uint8_t current_mask = readFaultMask(anOutError);
  if (anOutError != ESP_OK) {
    return;
  }

  current_mask |= aMask;
  setFaultMask(current_mask, anOutError);
}

void MAX31856::disableFault(uint8_t aMask, int &anOutError) {
  uint8_t current_mask = readFaultMask(anOutError);
  if (anOutError != ESP_OK) {
    return;
  }

  current_mask &= ~aMask;
  setFaultMask(current_mask, anOutError);
}

void MAX31856::setConvertType(bool isAutoConvert, int &anOutError) {
  esp_err_t ret;
  anOutError = ESP_OK;

  uint8_t cr0_val = readRegister(CR0::readAddress, ret);
  if (ret != ESP_OK) {
    anOutError = ret;
    return;
  }

  if (isAutoConvert) {
    cr0_val |= CR0::autoConversionMode; // Enable auto-convert mode
  } else {
    cr0_val &= ~CR0::autoConversionMode; // enable one-shot mode
  }

  writeRegister(CR0::writeAddress, cr0_val, ret);
  if (ret != ESP_OK) {
    anOutError = ret;
    return;
  }
}

void MAX31856::setTempFaultThreshholds(float low, float high, int &anOutError) {
  // Convert Celsius to linearized temperature fault threshold format
  // Per MAX31856 datasheet: Linearized temperature fault thresholds use
  // 0.0625°C per LSB To convert from °C to register value: divide by 0.0625 =
  // multiply by 16
  int16_t low_int = (int16_t)(low * 16.0f);
  int16_t high_int = (int16_t)(high * 16.0f);

  esp_err_t ret;
  anOutError = ESP_OK;

  // Write high threshold (big-endian)
  writeRegister(LTHFT::MSB::writeAddress, high_int >> 8, ret);
  if (ret != ESP_OK) {
    anOutError = ret;
    return;
  }
  writeRegister(LTHFT::LSB::writeAddress, high_int & 0xFF, ret);
  if (ret != ESP_OK) {
    anOutError = ret;
    return;
  }

  // Write low threshold (big-endian)
  writeRegister(LTLFT::MSB::writeAddress, low_int >> 8, ret);
  if (ret != ESP_OK) {
    anOutError = ret;
    return;
  }
  writeRegister(LTLFT::LSB::writeAddress, low_int & 0xFF, ret);
  if (ret != ESP_OK) {
    anOutError = ret;
    return;
  }
}

void MAX31856::setFaultMask(uint8_t aMask, int &anOutError) {
  writeRegister(FaultMask::writeAddress, aMask, anOutError);

  return;
}

uint8_t MAX31856::readFaultMask(int &anOuterror) {
  return readRegister(FaultMask::readAddress, anOuterror);
}

void MAX31856::oneShotTemperature(int &anOutError) {
  assert(false); // not functioning yet. An interupt for the DRDY pin is needed
                 // in order to read the data when the conversion is done.

  uint8_t val = readRegister(CR0::writeAddress, anOutError);
  if (anOutError != ESP_OK) {
    return;
  }

  val |= CR0::requestOneShot;
  writeRegister(CR0::writeAddress, val, anOutError);
  if (anOutError != ESP_OK) {
    return;
  }

  // Wait for conversion to complete by checking DRDY bit
  // or use timeout to prevent infinite loop
  //   uint32_t timeout = 0;
  //   const uint32_t MAX_TIMEOUT = 300; // ms
  //   while (timeout < MAX_TIMEOUT) {
  //     uint8_t status =
  //         readRegister(MAX31856_SR_REG,
  //                      anOutError); // TODO this is incorrect. There is no
  //                      DRDY
  //                                   // bit in the SR, it's a physical pin.
  //     if (anOutError != ESP_OK) {
  //       return;
  //     }

  //     if (!(status & 0x80)) { // DRDY bit cleared = conversion done
  //       break;
  //     }
  //     vTaskDelay(10 / portTICK_PERIOD_MS);
  //     timeout += 10;
  //   }

  //   if (timeout >= MAX_TIMEOUT) {
  //     ESP_LOGW(TAG, "One-shot conversion timeout");
  //     return false;
  //   }

  return;
}

float MAX31856::readCJTemp(int &anOutError) {
  // CJTH register: 14-bit signed value, bits 15:2
  uint16_t cj_temp = readRegister16(CJT::MSB::readAddress, anOutError);
  if (anOutError != ESP_OK) {
    return 0.0f; // Return on error
  }

  // Cold junction is 14-bit signed value in bits 15:2
  int16_t cj_signed = cj_temp >> 2; // Shift to get 14-bit value
  if (cj_signed & 0x2000) {         // Check bit 13 (sign bit)
    cj_signed |= 0xC000;            // Sign extend bits 15:14
  }
  return cj_signed * 0.015625f; // 0.015625°C per LSB
}

float MAX31856::readLTC(int &anOutError) {
  // LTCBH register: 19-bit signed value, bits 18:0
  uint32_t tc_temp = readRegister24(LTC::Byte2::readAddress, anOutError);
  if (anOutError != ESP_OK) {
    return 0.0f; // Return on error
  }

  tc_temp >>= 5; // bottom 5 bits are unused, now have 19-bit value

  // Handle sign extension for 19-bit signed value
  int32_t tc_signed = tc_temp;
  if (tc_signed & 0x40000) { // Check bit 18 (sign bit for 19-bit)
    tc_signed |= 0xFFF80000; // Sign extend bits 31:19
  }
  return tc_signed * 0.0078125f; // 0.0078125°C per LSB
}

uint8_t MAX31856::readFault(int &anOutError) {
  return readRegister(SR::readAddress, anOutError);
}

bool MAX31856::read(Result &anOutResult) {

  anOutResult.spi_success = true;  // Assume valid unless we find a fault
  anOutResult.error_code = ESP_OK; // Default to no error

  esp_err_t ret;

  if (!myConfig.auto_convert) {
    // If not in auto convert mode, trigger a one-shot temperature read
    oneShotTemperature(ret);
    if (ret != ESP_OK) {
      anOutResult.spi_success = false;
      anOutResult.error_code = ret;
      return false; // Return on error
    }
  }

  float cj_temp = readCJTemp(ret);
  if (ret != ESP_OK) {
    anOutResult.spi_success = false;
    anOutResult.error_code = ret;
    return false;
  }

  anOutResult.coldjunction_c = cj_temp;
  anOutResult.coldjunction_f = (cj_temp * 9.0f / 5.0f) + 32.0f;

  float tc_temp = readLTC(ret);
  if (ret != ESP_OK) {
    anOutResult.spi_success = false;
    anOutResult.error_code = ret;
    return false;
  }

  anOutResult.thermocouple_c = tc_temp;
  anOutResult.thermocouple_f = (tc_temp * 9.0f / 5.0f) + 32.0f;

  uint8_t fault_val = readFault(ret);
  if (ret != ESP_OK) {
    anOutResult.spi_success = false;
    anOutResult.error_code = ret;
    return false;
  }

  anOutResult.fault_bits = fault_val;
  anOutResult.fault.clear();

  // Log faults if requested and if any faults exist
  if (fault_val) {
    if (fault_val & SR::cjHigh)
      anOutResult.fault.push_back("Cold Junction High");
    if (fault_val & SR::cjLow)
      anOutResult.fault.push_back("Cold Junction Low");
    if (fault_val & SR::tcHigh)
      anOutResult.fault.push_back("Thermocouple High");
    if (fault_val & SR::tcLow)
      anOutResult.fault.push_back("Thermocouple Low");
    if (fault_val & SR::ovuv)
      anOutResult.fault.push_back("Over/Under Voltage");
    if (fault_val & SR::open)
      anOutResult.fault.push_back("Thermocouple Open");

    // Clear the faults
    if (!myConfig.auto_convert) {
      clearFault(ret);
      if (ret != ESP_OK) {
        anOutResult.spi_success = false;
        anOutResult.error_code = ret;
        return false;
      }
    }
  }

  return true;
}

void MAX31856::setColdJunctionOffset(float anOffsetCelsius, int &anOutError) {
  // CJTO register: signed 8-bit, 0.0625°C per LSB
  if (anOffsetCelsius < -8.0f)
    anOffsetCelsius = -8.0f;
  if (anOffsetCelsius > 7.9375f)
    anOffsetCelsius = 7.9375f;

  int8_t offset_int = static_cast<int8_t>(anOffsetCelsius / 0.0625f);

  esp_err_t ret;
  anOutError = ESP_OK;
  writeRegister(CJTO::writeAddress, static_cast<uint8_t>(offset_int), ret);
  if (ret != ESP_OK) {
    anOutError = ret;
    return;
  }
}

void MAX31856::setType(ThermocoupleType type, int &anOutError) {

  uint8_t value = 0;
  switch (type) {
  case ThermocoupleType::TYPE_B:
    value = CR1::thermocoupleTypeB;
    break;
  case ThermocoupleType::TYPE_E:
    value = CR1::thermocoupleTypeE;
    break;
  case ThermocoupleType::TYPE_J:
    value = CR1::thermocoupleTypeJ;
    break;
  case ThermocoupleType::TYPE_K:
    value = CR1::thermocoupleTypeK;
    break;
  case ThermocoupleType::TYPE_N:
    value = CR1::thermocoupleTypeN;
    break;
  case ThermocoupleType::TYPE_R:
    value = CR1::thermocoupleTypeR;
    break;
  case ThermocoupleType::TYPE_S:
    value = CR1::thermocoupleTypeS;
    break;
  case ThermocoupleType::TYPE_T:
    value = CR1::thermocoupleTypeT;
    break;
  case ThermocoupleType::TYPE_VMODE_G8:
    value = CR1::thermocoupleTypeVmodeG8;
    break;
  case ThermocoupleType::TYPE_VMODE_G32:
    value = CR1::thermocoupleTypeVmodeG32;
    break;
  default:
    anOutError = ESP_ERR_INVALID_ARG; // Invalid thermocouple type
    return;
  }

  esp_err_t ret;
  anOutError = ESP_OK;
  uint8_t val = readRegister(CR1::readAddress, ret);
  if (ret != ESP_OK) {
    anOutError = ret;
    return;
  }

  val |= value;
  writeRegister(CR1::writeAddress, val, ret);
  if (ret != ESP_OK) {
    anOutError = ret;
    return;
  }
}

void MAX31856::setAveragingMode(AveragingSamples aSamples, int &anOutError) {
  uint8_t anAveragingMode = 0;

  switch (aSamples) {
  case AveragingSamples::AVG_1:
    anAveragingMode = CR1::averagingMode1;
    break;
  case AveragingSamples::AVG_2:
    anAveragingMode = CR1::averagingMode2;
    break;
  case AveragingSamples::AVG_4:
    anAveragingMode = CR1::averagingMode4;
    break;
  case AveragingSamples::AVG_8:
    anAveragingMode = CR1::averagingMode8;
    break;
  case AveragingSamples::AVG_16:
    anAveragingMode = CR1::averagingMode16;
    break;
  default:
    anOutError = ESP_ERR_INVALID_ARG; // Invalid averaging mode
    return;
  }
  esp_err_t ret;
  uint8_t val = readRegister(CR1::readAddress, ret);
  if (ret != ESP_OK) {
    anOutError = ret;
    return;
  }

  if (aSamples == AveragingSamples::AVG_1) {
    // Check if averaging mode bits are already zero (bits 4-7)
    if ((val & CR1::averagingModeBits) == CR1::averagingMode1) {
      // Averaging mode bits are already zero, no need to update
      return;
    }
  }

  val |= anAveragingMode; // Set new averaging mode
  writeRegister(CR1::writeAddress, val, ret);
  if (ret != ESP_OK) {
    anOutError = ret;
    return;
  }

  return;
}

void MAX31856::setColdJunctionFaultThreshholds(float low, float high,
                                               int &anOutError) {
  // MAX31856 cold junction fault thresholds: -64°C to +127°C, 1°C per LSB
  if (low < -64.0f)
    low = -64.0f;
  if (low > 125.0f)
    low = 125.0f;
  if (high < -64.0f)
    high = -64.0f;
  if (high > 125.0f)
    high = 125.0f;

  int8_t low_int = static_cast<int8_t>(low);
  int8_t high_int = static_cast<int8_t>(high);

  esp_err_t ret;
  anOutError = ESP_OK;

  // Write directly as unsigned bytes (two's complement representation)
  writeRegister(CJHF::writeAddress, static_cast<uint8_t>(high_int), ret);
  if (ret != ESP_OK) {
    anOutError = ret;
    return;
  }
  writeRegister(CJLF::writeAddress, static_cast<uint8_t>(low_int), ret);
  if (ret != ESP_OK) {
    anOutError = ret;
    return;
  }
  return;
}

void MAX31856::clearFault(int &anOutError) {
  esp_err_t ret;
  anOutError = ESP_OK;

  uint8_t val = readRegister(CR0::readAddress, ret);
  if (ret != ESP_OK) {
    anOutError = ret;
    return;
  }

  val |= CR0::faultClear; // Set the fault clear bit
  writeRegister(CR0::writeAddress, val, ret);
  if (ret != ESP_OK) {
    anOutError = ret;
    return;
  }
}

} // namespace ESP_MAX318_THERMOCOUPLE