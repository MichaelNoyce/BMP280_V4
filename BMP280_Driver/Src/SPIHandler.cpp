#include "SPIHandler.hpp"

BMPStatus_t SPIHandler::transmit(uint8_t* data, int32_t size, int32_t timeout) {
    // Implementation of the transmit function
    // For example, using SPI to transmit data
    // This is a placeholder implementation
    if (HAL_SPI_Transmit(&hspi, data, size, timeout) == HAL_OK) {
        return BMP_OK;
    } else {
        return BMP_SPI_WRITE_ERROR;
    }
}

BMPStatus_t SPIHandler::receive(uint8_t* data, int32_t size, int32_t timeout) {
    // Implementation of the receive function
    // For example, using SPI to receive data
    // This is a placeholder implementation
    if (HAL_SPI_Receive(&hspi, data, size, timeout) == HAL_OK) {
        return BMP_OK;
    } else {
        return BMP_SPI_READ_ERROR;
    }
}