/* Includes ------------------------------------------------------------------*/
#include "platform_cfg.h"
#include "stdbool.h"
#include "cmsis_os.h"
#include "printf.h"

#include "drv_i2c.h"
#include "sensors.h"
#include "drv_gpio.h"
#include "drv_pwm.h"
#include "drv_mpu.h"
#include "drv_baro.h"
#include "drv_mag.h"
#include "drv_usart.h"
#include "imu.h"

#include "drv_msp.h"

static void Error_Handler(void);

/* Hardware handlers declaration */

/* Private variables ---------------------------------------------------------*/
osThreadId MainTaskHandle;

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void tick_task(void const * argument)
{
	const TickType_t xFrequency = 100;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
	  vTaskDelayUntil( &xLastWakeTime, xFrequency );

//	  LEDToggle( PIN_LED1 );
	}
};

void main_task(void * pvParameters)
{
	gpioInit();
	usartInit();
	printf("\n\r> Zaczynamy !!!");
	printf("\n\r> GPIO Initialised: OK");
	printf("\n\r> USART Initialised: OK");

	pwmInit();

	if( i2cInit() == HAL_OK ) printf("\n\r> I2Cx Initialised: OK");
	else 					  Error_Handler();

	if( sensorsAutodetect() != HAL_OK ) Error_Handler();

	LEDOn( PIN_LED0 );

	imuInit();

	LEDOn( PIN_LED1 );

	vTaskDelay( 1000 );

	printf("\n\r> MSP Start");
	mspInit( &UartHandle );

	osThreadDef(TickTask, tick_task, osPriorityIdle, 0, 128);
	osThreadCreate(osThread(TickTask), NULL);

	vTaskDelete(NULL);
}

int main(void)
{
	/* MCU Configuration----------------------------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	osThreadDef( MainTask, main_task, osPriorityNormal, 0, 256);
	osThreadCreate(osThread( MainTask ), NULL);

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	while(1){};
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();

	printf( "vApplicationMallocFailedHook\r\n" );
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* vApplicationStackOverflowHook() will only be called if
	configCHECK_FOR_STACK_OVERFLOW is set to either 1 or 2.  The handle and name
	of the offending task will be passed into the hook function via its
	parameters.  However, when a stack has overflowed, it is possible that the
	parameters will have been corrupted, in which case the pxCurrentTCB variable
	can be inspected directly. */
	printf("Stack Overflow in %s\r\n", pcTaskName);
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

static void Error_Handler(void)
{
	printf("\n\rERROR !!!");

  while (1)
  {
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);

  }
  /* USER CODE END 5 */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
