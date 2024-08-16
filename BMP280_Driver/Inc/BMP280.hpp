#ifndef BMP280_HPP
#define BMP280_HPP


#include <cstdint>
#include "SensorBase.hpp"
#include "BMP280_Structs.hpp"
#include "BMP280_Defines.hpp"
#include "SPIHandler.hpp"


class BMP280 : public SensorBase {
public:

    // Constructor that accepts a handler for a communication interface
    BMP280(SPIHandler& handler, BMP_Handle_Typedef& bmp, BMP_Init_Typedef& bmp_init) : handler(handler), bmp(bmp), bmp_init(bmp_init){
        // Initialize BMP280 sensor specific settings here
    }

    // Destructor
    ~BMP280() {
        // Cleanup BMP280 sensor resources here
    }

    // Public member variables
    BMP_Handle_Typedef& bmp; // BMP280 handler

    //======================== 0. SensorBase Virtual Functions =========================================

    // Override SensorBase's virtual functions
    void initialize() override {  
    }

    float readData() override {
        return 0.0f;
    }

    void calibrate() override {
    }

    void reset() override {
    }

    //======================== 1. BMP280 Init Function Prototypes =========================================  


    /**
    * Function Name void BMP280_Init_Preset_Mode(BMP_Opp_Mode_t BMP_MODE, BMP_Init_Typedef* BMP_InitStruct);
    *
    * @brief: Configures the paramaters of the BMP_InitStruct Mode to configure the BMP Sensor. Custom preset
    * 		   modes are defined in table 15. Page 19 of the reference manual
    *
    * @param: BMP_MODE	- the preset mode desired. This enumeration is defined in BMP_Opp_Mode_t
    *
    * @param BMP_IniStruct - empty Configuration struct
    *
    * @return: None
    */
    void BMP280_Init_Preset_Mode(BMP_Opp_Mode_t BMP_MODE, BMP_Init_Typedef* BMP_InitStruct);

    /**
    * Function Name void BMP280_Init_Custom(BMP_Init_Typedef* BMP_InitStruct, uint8_t mode, uint8_t osrsp, uint8_t osrst,uint8_t ifcoef, uint8_t t_stdby)
    *
    * @brief: configures paramaters of the BMP InitStruct using user specified values
    *
    * @param BMP_IniStruct - empty Configuration struct
    * @param mode - Device mode (Forced, Sleep, Normal)
    * @param osrsp - Pressure Oversample Value
    * @param osrst - Temperature Oversaple Value
    * @param ifcoef - IIR Filter Coefficient value
    * @param stdby - Standby Time value
    *
    * @return: None
    */
    void BMP280_Init_Custom(BMP_Init_Typedef* BMP_InitStruct, uint8_t mode, uint8_t osrsp, uint8_t osrst,uint8_t ifcoef, uint8_t t_stdby);

    /** 
    * @Param: Initialises the BMP Sensor and Configures the registers to the settings in the BMP_InitStruct
    *
    * @param: BMP_Init_Typedef* - pointer to BMP_InitStruct
    *
    * @return: BMPStatus_t - status of initialisation function
    *
    */
    BMPStatus_t BMP280_Init(BMP_Init_Typedef& BMP_InitStruct);

    /** 
    * Function Name BMPStatus_t BMP280_DeInit(void)
    *
    * @brief: De Intialise SPI Peripherals and set device to sleep mode
    *
    * @param: none
    *
    * @return BMPStatus_t - return status of function
    */
    BMPStatus_t BMP280_DeInit(void);

    //======================== 2. Reg Config Function Prototypes =========================================

    /**
    * Function Name BMPStatus_t BMP280_Reset(void);
    *
    * @brief: Performs a Device Software reset
    *
    * @param: None
    *
    * @return: BMPStatus_t - return status of function
    */
    BMPStatus_t BMP280_Reset(void);

    /** 
    * Function Name BMPStatus_t BMP280_Set_PowerMode(uint8_t mode);
    *
    * @brief: Configures the sensor Power mode
    *
    * @param mode: desired power mode setting
    *
    * @return: BMPStatus_t - return status of function
    */
    BMPStatus_t BMP280_Set_PowerMode(uint8_t mode);

    /**
    * Function Name BMPStatus_t BMP280_Set_Oversample(uint8_t osrs_t, uint8_t osrs_p);
    *
    * @brief: set the over-sampling parameter in the CTRL MEAS register for temperature and pressure
    *
    * @param: osrs_t - the desired over-sample parameter for pressure.
    * @param: osrs_p - the desired over-sample parameter for temperature.
    *
    * @return BMPStatus_t - return status of the function
    */
    BMPStatus_t BMP280_Set_Oversample(uint8_t osrs_t, uint8_t osrs_p);

    /**
    * Function Name BMPStatus_t BMP280_Set_IIR_Filter_Coeff(uint8_t iircoef);
    *
    * @brief: sets the IIR filter coefficients in the config register
    *
    * @param: iircoef - desired oversampling parameter
    *
    * @return BMPStatus_t - return status of the function
    */
    BMPStatus_t BMP280_Set_IIR_Filter_Coeff(uint8_t iircoef);

    /**
    * Function Name BMPStatus_t BMP280_Set_Standby_Time (uint8_t stdby);
    *
    * @brief: set the standby time between measurements for normal mode in the
    * 		   config register
    *
    * @param: stdby- bit representation of desired standby time
    *
    * @return BMPStatus_t - return status of function
    */
    BMPStatus_t BMP280_Set_Standby_Time (uint8_t stdby);

    /**
    * Function Name BMPStatus_t BMP280_Write_Register(uint8_t reg,int32_t size, uint8_t* data);
    *
    * @brief: Function to set values in a specified register on the BMP280. Register definitions
    * 		   are given in the BMPRegisterMap_t enumeration
    *
    * @param: reg - base address of the register to be written to
    *
    * @param: size - number of bytes to be writte
    *
    * @param data - pointer to array/memory address of data to be sent
    *
    * @return BMPStatus_t - return status of function
    */
    BMPStatus_t BMP280_Write_Register(uint8_t reg,int32_t size, uint8_t* data);

    /**
    * Function Name BMPStatus_t BMP280_Read_Register(uint8_t reg,int32_t size, uint8_t* data);
    *
    * @brief: reads a specified number of bytes from the device from the base register specified by
    * 		   the user.
    *
    * @param: reg - starting address of register to be read from
    *
    * @param: size - number of bytes to be read
    *
    * @param: data - pointer to a uin8_t buffer to hold data
    *
    * @return BMPStatus_t - return status of function
    */
    BMPStatus_t BMP280_Read_Register(uint8_t reg,int32_t size, uint8_t* data);

    //======================== 3. Data Read Function Prototypes=========================================

    /** 
    * Function Name: BMPStatus_t BMP280_Get_Measurements(uint32_t* adc_Temp,uint32_t* adc_Press);
    *
    * @brief: reads the temperature and pressure measurement registers from the device and combines
    * 		   them into 32 bit integer ADC values. Note: this function reads the values when the
    * 		   device is set to normal mode
    *
    * @param: adc_Temp - pointer to uint32_t variable to store the recorded Temperature value
    *
    * @param: adc_Press - pointer to uint32_t variable to store the recorded pressure value
    *
    * @return BMPStatus_t - return status of function
    */
    BMPStatus_t BMP280_Get_Measurements(uint32_t& adc_Temp,uint32_t& adc_Press);

    /** 
    * Function Name BMPStatus_t BMP280_Get_Status(BMP_Handle_Typedef* bmp);
    *
    * @brief: reads the status register on the device and updates the status variables
    * 		   of the bmp_handle_Typedef
    *
    * @param: bmp - pointer to BMP_HandleTypedef
    *
    * @return BMPStatus_t - return status of function
    */
    BMPStatus_t BMP280_Get_Status(BMP_Handle_Typedef* bmp);

    /** 
    * Function Name BMPStatus_t BMP280_Get_ID(uint8_t* dev_id);
    *
    * @brief: reads the id register of the device and returns the value
    *
    * @param: dev_id - pointer to variable to store the value read from the register
    *
    * @return BMPStatus_t - return status of function
    */
    BMPStatus_t BMP280_Get_ID(uint8_t* dev_id);

    /**
    * Function Name BMPStatus_t BMP280_Get_FacTrim( BMP280_trim_t *bmpt);
    *
    * @brief: Fetches the compensation parameters from calibration registers
    * 		   0 - 25. Converts values to uint16_t/int16_t and stores them
    * 		   in a BMP280_trim_t struct.
    * @param bmpt - pointer to a struct to hold the compensation parameters
    *
    * @return BMPStatus_t - return status of function
    */
    BMPStatus_t BMP280_Get_FactoryTrim( BMP280_trim_t *bmpt);

    //======================== 4. Measurement Function Prototypes=========================================

    /**  
    * Function Name BMPStatus_t BMP280_Force_Measure(uint32_t* temp,uint32_t* pressure);
    *
    * @brief: Triggers a temperature/ pressure measurement while the device is operating in forced mode.
    * 		   A Forced measurement occurs when the power mode is changed to force mode. This causes a
    * 		   conversion to occur. when the conversion has finnished, the device returns to sleep mode
    * 		   untill another measurement is forced
    * @param temp - pointer to 32 bit integer to store the temperature ADC value
    *
    * @param pressure - pointer to 32 bit integer to store the pressure ADC value.
    *
    * @return BMPStatus_t - return status of function
    */
    BMPStatus_t BMP280_Force_Measure(uint32_t& temp, uint32_t& pressure);

    /**  
    * Function Name BMPStatus_t BMP280_Get_Temp(uint32_t adc_TEMP);
    *
    * @brief: reads the temperature XLSB, LSB, MSB registers and combines the
    * 		   values to create a 20 bit integer.
    *
    * @param pressure - pointer to 32 bit integer to store the temperature ADC value.
    *
    * @return BMPStatus_t - return status of function
    *
    */
    BMPStatus_t BMP280_Get_Temp(uint32_t* adc_Temp);

    /** 
    * Function Name BMPStatus_t BMP280_Get_Pressure(uint32_t adc_Press);
    *
    * @brief: reads the pressure XLSB, LSB, MSB registers and combines the
    * 		   values to create a 20 bit integer.
    *
    * @param pressure - pointer to 32 bit integer to store the pressure ADC value.
    *
    * @return BMPStatus_t - return status of function
    *
    */
    BMPStatus_t BMP280_Get_Pressure(uint32_t* adc_Press);

    /** 
    * Function Name: int32_t BMP280_Compensate_Temp(int32_t T_val,int32_t* t_fine, BMP280_trim_t bmp_trim);
    *
    * @brief: Calculates the true temperature measurement using the Bosch BMP temperature compensation formula
    * 		   (page 21 - 23 of the datasheet). Also used to calculate the temperature calibration parameter t_fine
    * 		   for the pressure compensation algorithm.
    *
    * @param: T_val - ADC temperature
    *
    * @param: t_fine - pointer to variable to hold the paramter t_fine
    *
    * @param: bmp_trim - struct containing the calibration parameters
    *
    * @return int32_t representation of the compensated temperature. in degrees C
    * 		   (last 2 digits of the value represent the first 2 signifcant places)
    */
    int32_t BMP280_Compensate_Temp(int32_t T_val,int32_t& t_fine, BMP280_trim_t bmp_trim);

    /** 
    * Function Name: uint32_t BMP280_Compensate_Pressure(uint32_t P_val,int32_t t_fine,BMP280_trim_t bmp_trim);
    *
    * @brief: Calculates the true Atmospheric pressure measurement using the Bosch compensation formula
    *
    * @param P_val - ADC Pressure value
    *
    * @param t_fine - t_fine calibration parameter calculated from the Temperature Compensation formula
    *
    * @param bmp_trim - struct containing BMP calibration parameters
    *
    * @return uin32_t representation of compensated atmoshperic pressure in Pa
    */
    uint32_t BMP280_Compensate_Pressure(uint32_t P_val,int32_t t_fine,BMP280_trim_t bmp_trim);

    private:
        BMP_Init_Typedef& bmp_init;		//instance of BMP_Init_Typedef
        SPIHandler& handler; // Generic handler pointer for communication interface


};

#endif // BMP280_HPP