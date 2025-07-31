#pragma once
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "soc/gpio_struct.h"
#include <string>
#include <vector>

namespace ESP_MAX318_THERMOCOUPLE
{
    struct Result
    {
        float coldjunction_c;
        float coldjunction_f;
        float thermocouple_c;
        float thermocouple_f;
        std::vector<std::string> fault;
        uint8_t fault_bits;      // raw fault bits from the device
        int error_code;          // esp_err_t error code
        bool spi_success = true; // false if there was an error
    };

    class SPIError
    {
    public:
        static const uint8_t SPI_OK = 0;
        static const uint8_t SPI_ERR_NOT_INITIALIZED = 1;
        static const uint8_t SPI_ERR_INVALID_ARG = 2;
        static const uint8_t SPI_ERR_TIMEOUT = 3;
        static const uint8_t SPI_ERR_INVALID_STATE = 4;
        static const uint8_t SPI_ERR_NO_MEM = 5;
        static const uint8_t SPI_ERR_INVALID_SIZE = 6;
        static const uint8_t SPI_ERR_NOT_SUPPORTED = 7;
        static const uint8_t SPI_ERR_INVALID_RESPONSE = 8;
        static const uint8_t SPI_ERR_UNKNOWN = 9;
    };

    enum class AveragingSamples
    {
        AVG_1,
        AVG_2,
        AVG_4,
        AVG_8,
        AVG_16
    };

    struct MAX318Config // used as an interface for CreateDevice
    {
    };

    /**
     * @brief Base class for MAX318 devices
     *
     * This class provides a common interface for MAX318 devices, allowing SPI communication and basic register operations.
     * Derived classes should implement the read method to retrieve temperature data.
     */
    class MAX318_Base
    {
    public:
        MAX318_Base(spi_device_interface_config_t aSpiDeviceConfig);

        virtual bool read(Result &anOutResult) = 0;

        virtual bool configure(const MAX318Config *aConfig, const spi_device_handle_t &aHandle)
        {
            // Default implementation does nothing
            mySpiDeviceHandle = aHandle;
            return true;
        }

    protected:
        // generic SPI functions for any of this device class
        // anOutError is an esp_err_t error code
        void writeRegister(uint8_t anAddress, uint8_t someData, int &anOutError);
        uint8_t readRegister(uint8_t anAddress, int &anOutError);
        uint16_t readRegister16(uint8_t anAddress, int &anOutError);
        uint32_t readRegister24(uint8_t anAddress, int &anOutError);
        spi_device_handle_t mySpiDeviceHandle;
    };
}