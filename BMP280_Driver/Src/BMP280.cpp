#include "BMP280.hpp"
#include "hal_impl.hpp"


//========================= 1. Init Function Definitions =============================
         
/** 
* @brief: Initialises the sensor handler and performs power on/ begining procedure.
* 		   After this, the device is configured to user defined settings which are
* 		   then written to the registers. Note: A BMP_Init_Typedef struct must be initialised
* 		   and set before this function is called. THis can be done individually or by using the functions
* 		   BMP280_Init_Preset_Mode(), BMP280_Init_Custom()
* @param: BMP_InitStruct - struct containing the configuration values for initialising the sensor
* @retval BMPStatus_t - return value showing the status of the function for error handling
*/
BMPStatus_t BMP280::BMP280_Init(BMP_Init_Typedef& BMP_InitStruct){
{

//NOTE: Necessary Peripherals must be initialized in main before calling this function
//first attach an spi handler to the bmp sensor
//read device id to ensure device is online and functioning
    uint8_t dev_id = 0;
    uint8_t flag = BMP280_Get_ID(&dev_id);
    if(flag!= BMP_OK)
    {
        return static_cast<BMPStatus_t>(flag);
    }
    //check device ID matches
    if(dev_id != BMP280_ID)
    {
        return BMP_DEVICE_CHECK_ERROR;
    }
    
    //soft reset
    BMP280_Reset();
    
    //read factory trim values
    BMP280_Get_FactoryTrim(&(bmp.Factory_Trim));
    
    //get init data
    bmp.Init = BMP_InitStruct;
    
    //write data to ctrl_meas register
    uint8_t ctrlmeasbyte = bmp.Init.BMP_Pressure_OverSample |bmp.Init.BMP_Temperature_OverSample;
    uint8_t configbyte = bmp.Init.BMP_t_Standby | bmp.Init.BMP_IIR_FILTER_COEFFICIENTS;
    flag = BMP280_Write_Register(ctrl_meas,1,&ctrlmeasbyte);
    
    if(flag != BMP_OK)
    {
        return  static_cast<BMPStatus_t>(flag);
    }
    flag = BMP280_Write_Register(config,1,&configbyte);
    if(flag != BMP_OK)
    {
        return  static_cast<BMPStatus_t>(flag);
    }

    //in a seperate write, configure the power mode
    if(bmp.Init.BMP_Power_Mode != BMP280_CTRLMEAS_MODE_SLEEP)
    {
        flag = BMP280_Set_PowerMode(bmp.Init.BMP_Power_Mode);
        if(flag != BMP_OK)
        {
            return BMP_POWER_CONFIG_ERROR;
        }
    }

    return BMP_OK;
    
    }
}

//======================== 2. Reg Config Function Defintions =========================================

/** 
* @brief Function to Read data from an 8-bit register in the BMP280. Setting the size variable to
* 		   A number > 1 configures the device for burst read.
* @param reg
* @param size
* @param data
*
* @retval none
*/
BMPStatus_t BMP280::BMP280_Read_Register(uint8_t reg, int32_t size, uint8_t* data)
{
    uint8_t temp = reg | BMP280_SPI_READ;
    if (handler.transmit(&temp, 1, 100) == BMP_OK)
    {
        if (handler.receive(data, size, 100) != BMP_OK)
        {
            return BMP_SPI_READ_ERROR;
        }
    }
    else
    {
        return BMP_SPI_WRITE_ERROR;
    }

    return BMP_OK;
}

/** 
* @brief Writes data to an 8-bit register in the BMP280. Burst write is allowed and
* 	      happens by setting the variable size > 1.
*
* @param reg
* @param size
* @param data
*
* @retval BMPStatus_t
*/
BMPStatus_t BMP280::BMP280_Write_Register(uint8_t reg,int32_t size, uint8_t* data)
{
    
    uint8_t temp = reg & BMP280_SPI_WRITE;
    
    if(handler.transmit(&temp, 1, 100) != BMP_OK)
    {
        return BMP_SPI_WRITE_ERROR;
    }
    
    if(handler.transmit(data, size, 100) != BMP_OK)
    {
        return BMP_SPI_WRITE_ERROR;
    }
    
    return BMP_OK;
}


/**
* @brief return the Factory-set device ID
*
* @param dev_id
*
* @retval BMPStatus_t
*/
BMPStatus_t  BMP280::BMP280_Get_ID(uint8_t* dev_id)
{
    return BMP280_Read_Register(id,1,dev_id);
}

/** 
* @brief Reset the device. NB: Must include a 4ms delat after writing reset value
*
* @param none
*
* @retval BMPStatus_t
*/

BMPStatus_t  BMP280::BMP280_Reset(void)
{
    uint8_t data = BMP280_Soft_Reset&0xFF;
    if(BMP280_Write_Register(Sreset,1,&data) != BMP_OK)
    {
        return BMP_RESET_ERROR;
    }

    //HAL_Delay(4);<- this is a delay function that must be implemented in the main file

    return BMP_OK;
}

/**
* @brief configure power mode via SPI
*
* @param mode
*
* @retval BMPStatus_t
*/
BMPStatus_t BMP280::BMP280_Set_PowerMode(uint8_t mode)
{
    uint8_t reg =0;
    if(BMP280_Read_Register(ctrl_meas,1,&reg) != BMP_OK)
    {
        return BMP_SPI_READ_ERROR;
    }
    //clear the old settings
    reg &= 0b11111100;
    //configure the new mode
    reg |= mode;
    if(BMP280_Write_Register(ctrl_meas,1,&reg) != BMP_OK)
    {
        return BMP_Config_Error;
    }
    return BMP_OK;
}

/** 
* @brief configure oversample parameters
*
* @param osrs_t
* @param osrs_p
*
* @retval BMPStatus_t
*/
BMPStatus_t BMP280::BMP280_Set_Oversample(uint8_t osrs_t, uint8_t osrs_p)
{
    uint8_t reg = 0;
    if(BMP280_Read_Register(ctrl_meas,1,&reg) != BMP_OK)
    {
        return BMP_SPI_READ_ERROR;
    }
    //clear the old settings
    reg &= 0b11;
    //configure the new mode
    reg |= (osrs_t |osrs_p);
    if(BMP280_Write_Register(ctrl_meas,1,&reg) != BMP_OK)
    {
        return BMP_Config_Error;
    }
    return BMP_OK;
}

/**
* @brief configure IIR filter coefficients
*
* @param iircoef
*
* @retval BMPStatus_t
*/
BMPStatus_t BMP280::BMP280_Set_IIR_Filter_Coeff(uint8_t iircoef)
{
    uint8_t reg,mask = 0b11100;
    if(BMP280_Read_Register(config,1,&reg) != BMP_OK)
    {
        return BMP_SPI_READ_ERROR;
    }
    //clear the old settings
    reg &= ~mask;
    //configure the new mode
    reg |= iircoef;
    if(BMP280_Write_Register(config,1,&reg) != BMP_OK)
    {
        return BMP_Config_Error;
    }
    return BMP_OK;
}

/**
* @brief configure standby time
*
* @param stdby
*
* @retval BMPStatus_t
*/
BMPStatus_t BMP280::BMP280_Set_Standby_Time (uint8_t stdby)
{
    uint8_t reg,mask = 0b11100000;
    if(BMP280_Read_Register(config,1,&reg) != BMP_OK)
    {
        return BMP_SPI_READ_ERROR;
    }
    //clear the old settings
    reg &= ~mask;
    //configure the new mode
    reg |= stdby;
    if(BMP280_Write_Register(config,1,&reg) != BMP_OK)
    {
        return BMP_Config_Error;
    }
    return BMP_OK;
}

//======================== 4. Data Read Function Definitions =========================================

/*
* @brief IM Copy and Conversion status read function
*
* @param bmp
*
* @retval BMPStatus_t
*/
BMPStatus_t BMP280::BMP280_Get_Status(BMP_Handle_Typedef* bmp)
{
    //read contents of register
    uint8_t statusbyte = 0;
    if(BMP280_Read_Register(status,1,&statusbyte) != BMP_OK)
    {
        return BMP_SPI_READ_ERROR;
    }
    bmp->IM_Status = static_cast<BMP_IM_Status_t>(statusbyte&0b1);
    bmp->M_Status = static_cast<BMP_Measurement_Status_t>(statusbyte&0b1<<3);
    return BMP_OK;
}

/** 
* @brief Read the calibration parameters using burst read registers 0 - 25
*
* @param bmpt
*
* @retval BMPStatus_t
*/
BMPStatus_t BMP280::BMP280_Get_FactoryTrim( BMP280_trim_t *bmpt)
{
    uint8_t trim[24]= {0};
    BMP280_Read_Register(calib_0,24,trim);
    bmpt->dig_T1 = 			((uint8_t)trim[1]<<8) |((uint8_t)trim[0]);
    bmpt->dig_T2 = (int16_t)((uint8_t)trim[3]<<8) |((uint8_t)trim[2]);
    bmpt->dig_T3 = (int16_t)((int8_t)trim[5]<<8) |((int8_t)trim[4]);
    /*Preaseure Measurement Readouts */
    bmpt->dig_P1 = 			((uint8_t)trim[7]<<8) |((uint8_t)trim[6]);
    bmpt->dig_P2 = (int16_t)(((uint8_t)trim[9]<<8) |((uint8_t)trim[8]));
    bmpt->dig_P3 = (int16_t)(((uint8_t)trim[11]<<8)|((uint8_t)trim[10]));
    bmpt->dig_P4 = (int16_t)(((uint8_t)trim[13]<<8)|((uint8_t)trim[12]));
    bmpt->dig_P5 = (((uint8_t)trim[15]<<8)|((uint8_t)trim[14]));
    bmpt->dig_P6 = (int16_t)(((uint8_t)trim[17]<<8)|((uint8_t)trim[16]));
    bmpt->dig_P7 = (int16_t)(((uint8_t)trim[19]<<8)|((uint8_t)trim[18]));
    bmpt->dig_P8 = (int16_t)(((uint8_t)trim[21]<<8)|((uint8_t)trim[20]));
    bmpt->dig_P9 = (int16_t)(((uint8_t)trim[23]<<8)|((uint8_t)trim[22]));
    return BMP_OK;
}

//======================== 5. Measurement Function Definitions =========================================

BMPStatus_t BMP280::BMP280_Force_Measure(uint32_t& temp, uint32_t& pressure)
{
	//write forced mode to register
	BMP280_Set_PowerMode(BMP280_CTRLMEAS_MODE_FORCED);
	//wait for device to finish converting
	uint8_t status;
	BMP280_Read_Register(ctrl_meas,1,&status);
	while((status&0b11) != 0)
	{
		BMP280_Read_Register(ctrl_meas,1,&status);
	}
	//wait for control register to finish
	BMP280_Get_Measurements(temp,pressure);
	return BMP_OK;
}


/*
 * @brief Read measurement data (any mode)
 *
 * @param adc_Temp
 * @param adc_Press
 *
 * @retval BMPStatus_t
 */

BMPStatus_t BMP280::BMP280_Get_Measurements(uint32_t& adc_Temp, uint32_t& adc_Press)
{
	uint8_t data[6] = {0};
	BMP280_Read_Register(press_msb,6,data);
	adc_Temp = ((data[3]&0xFF)<<16)|((data[4]&0xFF)<<8)|((data[5]&0xF0));
	adc_Temp= adc_Temp>> 4;
	uint32_t temp= ((data[0]&0xFF)<<16)|((data[1]&0xFF)<<8)|((data[2]&0xFF))/16;
	adc_Press = temp/16;
	//convert to 20 bit unsigned integers
	return BMP_OK;
}

/*
 * @brief: function to read the 3 temperature registers and combine the data into a 20bit long signed integer.
 * 		   Temp is stored as 3 unsigned integers 8 bits long in registers 0xFA - 0xFB on the BMP280 Sensor. Data
 * 		   is Read MSB first. These 3 bytes represent the raw temperature reading and must be compensated using the
 * 		   algorithm outlined in the data sheet
 *
 * @param: adc_temp - pointer to an signed32_t integer to hold the converted value of temperature
 *
 * @return: BMPStatus_t status of the conversion for error handling
 */

BMPStatus_t BMP280::BMP280_Get_Temp(uint32_t* adc_Temp)
{
	uint8_t Ttemp[3] = {0};
	//read the registers MSB first
	if(BMP280_Read_Register(temp_msb,3,Ttemp) != BMP_OK)
	{
		return BMP_TEMP_READ_ERROR;
	}
	*adc_Temp = ((Ttemp[0]&0xFF)<<16)|((Ttemp[1]&0xFF)<<8)|((Ttemp[2]&0xF0));
	*adc_Temp= *adc_Temp>> 4;

	return BMP_OK;
}


/*
 * @brief Temperature Compensation algorithm
 *
 * @param T_val
 * @param t_fine
 * @param bmp_trim
 *
 * @retval int32_t
 */
int32_t  BMP280::BMP280_Compensate_Temp(int32_t T_val,int32_t& t_fine, BMP280_trim_t bmp_trim)
{
	//compensate Temperature from datasheet
	int32_t var1 = (((T_val>>3)- ((int32_t)bmp_trim.dig_T1<<1))*((int32_t)bmp_trim.dig_T2))>>11;
	int32_t var2 =  (((((T_val>>4) - ((int32_t)bmp_trim.dig_T1)) * ((T_val>>4) - ((int32_t)bmp_trim.dig_T1))) >> 12)*((int32_t)bmp_trim.dig_T3)) >> 14;
	int32_t temp = var1+var2; //for storage in global variable
	t_fine = temp;
	return (temp*5 +128)/256;
}

/*
 * @brief: function to read the 3 pressure registers and combine the data into a 20bit long signed integer.
 * 		   Pressure is also is stored as 3 unsigned integers 8 bits long in registers 0xF7 - 0xF9 on the BMP280 Sensor. Data
 * 		   is Read MSB first. These 3 bytes represent the raw pressure reading and must be compensated using the
 * 		   algorithm outlined in the data sheet
 *
 * @param: adc_Press - pointer to an signed32_t integer to hold the converted value of pressure
 *
 * @return: BMPStatus_t status of the conversion for error handling
 */

BMPStatus_t BMP280::BMP280_Get_Pressure(uint32_t* adc_Press)
{
	uint8_t Ptemp[3] = {0};
	//read the registers MSB first
	if(BMP280_Read_Register(press_msb,3,Ptemp) != BMP_OK)
	{
		return BMP_PRESS_READ_ERROR;
	}
	uint32_t temp= ((Ptemp[0]&0xFF)<<16)|((Ptemp[1]&0xFF)<<8)|((Ptemp[2]&0xFF))/16;
	*adc_Press = temp/16;
	return BMP_OK;
}


/*
 * @brief Pressure compensation formula
 *
 * @param P_val
 * @param t_fine
 * @param bmp_trim
 *
 * @retval uint32_t
 */
uint32_t  BMP280::BMP280_Compensate_Pressure(uint32_t P_val,int32_t t_fine,BMP280_trim_t bmp_trim)
{
	//Compensation formula
	int32_t var1 = (int64_t)t_fine - 128000;
	int64_t var2 = var1*var1*((int64_t)(bmp_trim.dig_P6));
	var2 = var2 + (((int64_t)bmp_trim.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)bmp_trim.dig_P3)>>8) + ((var1 * (int64_t)bmp_trim.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bmp_trim.dig_P1)>>33;
	//check for divide by 0 error
	if(var1 == 0)return 0;
	int64_t P = 1048576 - (int32_t)P_val;
	P = (((P<<31)-var2)*3125)/var1;
	var1 = (((int64_t)(bmp_trim.dig_P9)) * (P>>13) * (P>>13)) >> 25;
	var2 = (((int64_t)(bmp_trim.dig_P8)) * P) >> 19;
	P = ((P + var1 + var2) >> 8) + (((int64_t)(bmp_trim.dig_P7))<<4);
	return (uint32_t)P;
}
