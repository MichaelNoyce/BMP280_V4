#pragma once 

#include "stm32l4xx_hal.h" // NB: this MUST be include before the SPI header file
#include "stm32l4xx_hal_spi.h"
#include "BMP280_Structs.hpp"


class SPIHandler {

    // Constructor that accepts a handler for a communication interface
    SPIHandler(SPI_HandleTypeDef& hspi) : hspi(hspi){
    }

    public:
        BMPStatus_t transmit(uint8_t* data, int32_t size, int32_t timeout);
        BMPStatus_t receive(uint8_t* data, int32_t size, int32_t timeout);

    private: 
        SPI_HandleTypeDef hspi;
};