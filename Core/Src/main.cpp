#include "main.hpp"
#include "hal_interface.hpp"
#include "hal_impl.hpp"
#include "BMP280.hpp"

#pragma GCC diagnostic ignored "-Wwrite-strings"
#pragma GCC diagnostic ignored "-Wformat"

#ifdef __cplusplus 
extern "C" {
#endif
#include "init.h"
#include "string.h"
#include <stdarg.h>
#include "stdio.h"

//FreeRTOS includes
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>
#include <event_groups.h>
#ifdef __cplusplus 
}
#endif

//======================== 0. Peripheral Handles ============================================================
UART_HandleTypeDef hlpuart1;
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
HAL_Impl halImpl;

//======================== 0. END ============================================================================

//======================== 1. Function Prototypes ============================================================
// Function pointers for HAL functions
static void LED_task(void *args); 
//======================== 1. END ============================================================================

int main(void) {
    
//======================== 1. SYSTEM INIT & CLOCK CONFIG ========================//
    HAL_Init();
    SystemClock_Config();
    setupHAL(&halImpl);

    // Initialize Peripherals
    halImpl.MX_SPI1_Init();
    halImpl.MX_DMA_Init();
	// Init_Control_Pins();

	//printmsg("SHARC BUOY STARTING! \r\n");    
//=================================== 1. END ====================================//

//======================== 2. BMP280 Polling Implementation =====================================//

    // Initialize the BMP280 sensor
    SPIHandler spiHandler(hspi1);
    BMP_Handle_Typedef bmpHandle;
    BMP_Init_Typedef bmpInit;

    // Environmental Sensor Init Routine
	BMP_Init_Typedef BMP_InitStruct = { 0 };
	
    //configure device for environmental sensing in forced mode
	BMP_InitStruct.BMP_Pressure_OverSample = BMP280_CTRLMEAS_OSRSP_OS_1;
	BMP_InitStruct.BMP_Temperature_OverSample = BMP280_CTRLMEAS_OSRST_OS_1;
	BMP_InitStruct.BMP_IIR_FILTER_COEFFICIENTS = BMP280_CONFIG_FILTER_COEFF_OFF;
	BMP_InitStruct.BMP_Power_Mode = BMP280_CTRLMEAS_MODE_FORCED;

    // Create the BMP280 object
    BMP280 bmp280(spiHandler, bmpHandle, bmpInit);

    // Environmental data struct


    // Initialize the BMP280 sensor
    if (bmp280.BMP280_Init(BMP_InitStruct) == BMP_OK) {
		printmsg("Environmental Sensor Online!\r\n");
		
        //create variables
		uint32_t temp, press;
		int32_t t_fine;
        EnvData_t BMP280Data;


		for (int i = 0; i < 2; ++i) //60 second compensation period for averaging
		{
			
            bmp280.BMP280_Force_Measure(temp, press);		//trigger conversion
			BMP280Data.env_Temp = bmp280.BMP280_Compensate_Temp(temp, t_fine, 
            bmpHandle.Factory_Trim);			//compensate temperature
			BMP280Data.atm_Press = bmp280.BMP280_Compensate_Pressure(press, t_fine,
					bmpHandle.Factory_Trim) / 256;	//compensate pressure
			
            halImpl.HAL_Delay(1000);

		}

		printmsg("Temp = %ld C \t Pressure = %lu Pa \r\n", BMP280Data.env_Temp,
				BMP280Data.atm_Press);

//=================================== 2. END ====================================//

//=================================== 3. BMP280 DMA Test ========================//



    //configure device for environmental sensing in normal mode
	BMP_InitStruct.BMP_Pressure_OverSample = BMP280_CTRLMEAS_OSRSP_OS_1;
	BMP_InitStruct.BMP_Temperature_OverSample = BMP280_CTRLMEAS_OSRST_OS_1;
	BMP_InitStruct.BMP_IIR_FILTER_COEFFICIENTS = BMP280_CONFIG_FILTER_COEFF_OFF;
	BMP_InitStruct.BMP_Power_Mode = BMP280_CTRLMEAS_MODE_FORCED;
//Implement DMA for SPI

    // Enable the DMA for SPI
    HAL_SPI_Receive_DMA(&hspi1, (uint8_t*)bmpHandle.Factory_Trim.dig_T1, 24);
    
//=================================== 3. END ====================================//

//=================================== 4. FreeRTOS Task Creation ========================//
    // Create a blinking LED task for the on-board LED.
    static StaticTask_t exampleTaskTCB;
    static StackType_t exampleTaskStack[ 512 ];

    // Create the blink task
    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;  // 500 ms delay

    TaskHandle_t returnStatus = xTaskCreateStatic( LED_task,
                                  "Blink_LED",
                                  configMINIMAL_STACK_SIZE,
                                  (void*)xDelay,
                                  configMAX_PRIORITIES - 1U,
                                  &( exampleTaskStack[ 0 ] ),
                                  &( exampleTaskTCB ) );

    
    printmsg("Task creation status: %d\r\n", returnStatus);

    
    // Start scheduler 
    vTaskStartScheduler();

    }

//=================================== 4. END ====================================//

}

//Debug Print
void printmsg(char *format,...) {
    char str[80];

    /*Extract the the argument list using VA apis */
    va_list args;
    va_start(args, format);
    vsprintf(str, format,args);
    halImpl.HAL_UART_Transmit(&hlpuart1,(uint8_t *)str, strlen(str),HAL_MAX_DELAY);
    va_end(args);
}

static void LED_task(void *pvParameters) {

    TickType_t pxDelay = (TickType_t)pvParameters;

    for(;;) {
        halImpl.HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        vTaskDelay(pxDelay);

    }
}

#if ( configCHECK_FOR_STACK_OVERFLOW > 0 )

    void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                        char * pcTaskName )
    {
        /* Check pcTaskName for the name of the offending task,
         * or pxCurrentTCB if pcTaskName has itself been corrupted. */
        ( void ) xTask;
        ( void ) pcTaskName;
    }

#endif /* #if ( configCHECK_FOR_STACK_OVERFLOW > 0 ) */



