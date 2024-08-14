/**
 * @file BMP280_Structs.hpp
 * @brief This file contains the structures and enumerations used in the BMP280 driver.
 */

#ifndef BMP280_STRUCTS_HPP
#define BMP280_STRUCTS_HPP

#include <stdint.h>

/**
 * @struct BMP280_trim_t
 * @brief Structure used to hold the values stored in the compensation registers: calib0 - calib25
 *
 * This structure must be used with the function BMPStatus_t BMP280_Get_FactoryTrim(BMP280_trim_t *bmpt).
 */
typedef struct{
	uint16_t 	dig_T1;
	int16_t 	dig_T2;
	int16_t 	dig_T3;
	uint16_t 	dig_P1;
	int16_t 	dig_P2;
	int16_t 	dig_P3;
	int16_t 	dig_P4;
	int16_t 	dig_P5;
	int16_t 	dig_P6;
	int16_t 	dig_P7;
	int16_t 	dig_P8;
	int16_t 	dig_P9;

}BMP280_trim_t;

/** 
 * @struct BMPRegisterMap_t
 *
 * @brief This enumerations maps the entire memory layout of the BMP280 sensor.
 * Each register is 8 bits long and each enumeration represents the base address
 * of that register. A full description of the memory map is
 * privided in the reference manual pages 24 - 27. please note that there are
 * 26 registers for the trim parameters named calib_0 - calib_25. The base address (calib0)
 * is provided and the nth register (calib_n) can be access by adding n to the base address.
 */

typedef enum
{
	temp_xlsb = 0xFC,
	temp_lsb = 0xFB,
	temp_msb = 0xFA,
	press_xlsb = 0xF9,
	press_lsb =0xF8,
	press_msb = 0xF7,
	config = 0xF5,
	ctrl_meas = 0xF4,
	status = 0xF3,
	Sreset = 0xE0,
	id = 0xD0,
	calib_0 = 0x88

} BMPRegisterMap_t;

/** 
 * @struct BMP_Opp_Mode_t
 *
 * @brief Enumeration of preset modes described in Table 15. Page 19 of the reference manual. By using one of these
 * values in the function BMP280_Init_Preset_Mode(BMP_Opp_Mode_t BMP_MODE, BMP_Init_Typedef* BMP_InitStruct).
 * Settting the value of BMP_MODE automatically configures BMP_InitStruct with the settings that allow the device
 * to opperate in that mode
 *
 * 	MODE NAME:...................MODE:............Standby Time:......OSRSP:......OSRST:.......IIRC:
 * 	HandHeld_LP..................NORMAL...........62.5ms.............16..........2............4
 * 	HandHeld_Dynamic.............NORMAL...........0.5ms..............4...........1............16
 * 	WEATHER_Monitoring...........FORCED...........X..................1...........1............OFF
 *
 */

typedef enum
{
	BMP_SPI_WRITE_ERROR,
	BMP_SPI_READ_ERROR,
	BMP_RESET_ERROR,
	BMP_DEVICE_CHECK_ERROR,
	BMP_Config_Error,
	BMP_TEMP_READ_ERROR,
	BMP_PRESS_READ_ERROR,
	BMP_POWER_CONFIG_ERROR,
	BMP_TIMEOUT,
	BMP_OK,
	BMP_ERROR,
	BMP_WRITE_ERROR,
	BMP_Converting,
	BMP_Conversion_Complete,

}BMPStatus_t;

/**
 * @enum BMP_Opp_Mode_t
 * 
 * @brief Preset modes
 * 
 */
typedef enum
{
	HandHeld_LP,
	HandHeld_Dynamic,
	Weather_Monitoring
}BMP_Opp_Mode_t;

/** 
 * @enum BMP_Measuremment_Status_t
 *
 * @brief Values that represent the conversion status of the sensor
 */
typedef enum
{
	BMP_Measurement_Busy = 0b1000,
	BMP_Measurement_Complete = 0

}BMP_Measurement_Status_t;

/** 
 * @enum BMP_IM_Status_t
 *
 * @brief Indicates the status of data being copied to the shadow registers
 */
typedef enum
{
	BMP_Data_Shadowing_Busy = 0b1,
	BMP_Data_Shadowing_Complete = 0

}BMP_IM_Status_t;

/** 
 * @struct BMP_Init_Typedef
 *
 * @brief A struct representing the sensor on the microcontroller side.
 * This will contain the settings for configuring the sensor.
 *
 * @note
 * This struct must be initialized and configured before the
 * sensor is initialized using the Macros shown in section 3.
 * 
 * @note
 * The following needs to be configured:
 *
 * BMP_Power_Mode:	The power mode of the device (SLEEP, FORCED, NORMAL)
 * BMP_Pressure/Temperature_OverSample: Oversample settings for temp and pressure (OSRSP/OSRST)
 * BMP_t_Standby: Set the output rate by setting the standby time between samples (BMP280_CONFIG_tsb)
 * BMP_IIR_FILTER_COEFFICIENTS: set the filter coefficients for the IIR Filter (BMP280_CONFIG_FILTER_COEFF)
 */

typedef struct
{
	uint8_t BMP_Power_Mode;
	uint8_t BMP_Pressure_OverSample;
	uint8_t BMP_Temperature_OverSample;
	uint8_t BMP_t_Standby;
	uint8_t BMP_IIR_FILTER_COEFFICIENTS;
} BMP_Init_Typedef;

/** 
 * @struct BMP_Handle_Typedef
 *
 * @brief This structure virtualises the BMP instance on the microcontroller side
 * all communications, configuration settings and compensation values are
 * faciliated through this handler.
 *
 * @note
 * Init:				Instance of BMP_Init_Typedef
 * IM_Status:			Status of Shadow register copying process
 * M_Status:			Status of ADC conversion
 * Factory_Trim:		Structure to hold compensation values
 * bmp_spi:				Pointer to spi handle typedef
 */
typedef struct
{
	BMP_Init_Typedef Init;
	BMP_IM_Status_t IM_Status;
	BMP_Measurement_Status_t M_Status;
	BMP280_trim_t Factory_Trim;
}BMP_Handle_Typedef;

#endif // BMP280_STRUCTS_HPP