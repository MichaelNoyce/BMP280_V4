/**
 * @file BMP280_Defines.hpp
 * @brief This file contains the definitions used in the BMP280 driver.
 */

#pragma once

//SPI Definitions
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOA

/*
 * USE_SPI_x maps the spi handler to the correct SPI POrt. On the stm32l476RG, there are 3 spi peripherals.
 * Note: set x to a value from 1 - 3 otherwise the code will not work
 */
#define USE_SPI_1					//SPI Instance

#ifdef USE_SPI_1
#define BMP_SPI_PORT SPI1
#endif

#ifdef USE_SPI_2
#define BMP_SPI_PORT SPI2
#endif

#ifdef USE_SPI_3
#define BMP_SPI_PORT SPI3
#endif

//SPI Pin Definitions

#define BMP_SPI_SCLK_PIN GPIO_PIN_5
#define BMP_SPI_MISO_PIN GPIO_PIN_6
#define BMP_SPI_MOSI_PIN GPIO_PIN_7

#define BMP_SPI_GPIO_PORT GPIOA

#define BMP280_I2C_ADDRESS1 0x76	//FOR I2C use
#define BMP280_I2C_ADDRESS2 0x77

//BMP CTRLMEAS Register Definitions

//Pressure Oversample values
#define BMP280_CTRLMEAS_OSRSP_SKIP  0 << 2
#define BMP280_CTRLMEAS_OSRSP_OS_1  1 << 2
#define BMP280_CTRLMEAS_OSRSP_OS_2  2 << 2
#define BMP280_CTRLMEAS_OSRSP_OS_4  3 << 2
#define BMP280_CTRLMEAS_OSRSP_OS_8  4 << 2
#define BMP280_CTRLMEAS_OSRSP_OS_16 5 << 2

//Temperature Oversample values
#define BMP280_CTRLMEAS_OSRST_SKIP  0 << 5
#define BMP280_CTRLMEAS_OSRST_OS_1  1 << 5
#define BMP280_CTRLMEAS_OSRST_OS_2  2 << 5
#define BMP280_CTRLMEAS_OSRST_OS_4  3 << 5
#define BMP280_CTRLMEAS_OSRST_OS_8  4 << 5
#define BMP280_CTRLMEAS_OSRST_OS_16 5 << 5

//Power Mode Settings
#define BMP280_CTRLMEAS_MODE_SLEEP  0
#define BMP280_CTRLMEAS_MODE_FORCED 1
#define BMP280_CTRLMEAS_MODE_NORMAL 3

//CONFIG Register Definitions

//Standby Time configuration
#define BMP280_CONFIG_tsb_0_5  0 << 5
#define BMP280_CONFIG_tsb_62_5 1 << 5
#define BMP280_CONFIG_tsb_125  2 << 5
#define BMP280_CONFIG_tsb_250  3 << 5
#define BMP280_CONFIG_tsb_500  4 << 5
#define BMP280_CONFIG_tsb_1000 5 << 5
#define BMP280_CONFIG_tsb_2000 6 << 5
#define BMP280_CONFIG_tsb_4000 7 << 5

//IIR Filter Coeeficients
#define BMP280_CONFIG_FILTER_COEFF_OFF 0 << 2
#define BMP280_CONFIG_FILTER_COEFF_2   1 << 2
#define BMP280_CONFIG_FILTER_COEFF_4   2 << 2
#define BMP280_CONFIG_FILTER_COEFF_8   3 << 2
#define BMP280_CONFIG_FILTER_COEFF_16  4 << 2

//SPI Settings
#define BMP280_CONFIG_SPI3_EN  1
#define BMP280_CONFIG_SPI3_DIS 0

//Constants and Litterals

#define BMP280_CALIB_DATA_LEN 24	//Number of registers to be read from
#define BMP280_Soft_Reset 0xB6		//Causes device to reset when written to the reset register
#define BMP280_SPI_READ 0x80		//SPI READ COMMAND
#define BMP280_SPI_WRITE 0x7F		//SPI WRITE COMMAND
#define BMP280_ID 0x58			//Device ID
#define __SPI_CS_ENABLE()	(SPI_CS_GPIO_Port->ODR) &= ~ SPI_CS_Pin	//SETS CHIP SELECT ENABLED
#define __SPI_CS_DISABLE()	(SPI_CS_GPIO_Port->ODR) |=  SPI_CS_Pin	//SETS CHIP SELECT DISABLED