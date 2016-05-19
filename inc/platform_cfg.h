/*
 * platform_cfg.h
 *
 *  Created on: 22 mar 2016
 *      Author: lucja
 */
#ifndef PLATFORM_CFG_H_
#define PLATFORM_CFG_H_

#include "common.h"
#include "drv_gpio.h"

#define LED0_GPIO   					GPIOB
#define LED0_PIN    					GPIO_PIN_3
#define LED0_GPIO_CLK_ENABLE()      	__HAL_RCC_GPIOB_CLK_ENABLE()

#define LED1_GPIO   					GPIOB
#define LED1_PIN    					GPIO_PIN_4
#define LED1_GPIO_CLK_ENABLE()      	__HAL_RCC_GPIOB_CLK_ENABLE()

#define BEEP_GPIO   					GPIOA
#define BEEP_PIN    					GPIO_PIN_12
#define BEEP_GPIO_CLK_ENABLE()      	__HAL_RCC_GPIOA_CLK_ENABLE()

#define BARO_XCLR_GPIO   				GPIOC
#define BARO_XCLR_PIN    				GPIO_PIN_13
#define BARO_XCLR_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOC_CLK_ENABLE()

#define BARO_EOC_GPIO    				GPIOC
#define BARO_EOC_PIN     				GPIO_PIN_14
#define BARO_EOC_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOC_CLK_ENABLE()

#define MPU_INT_GPIO    				GPIOB
#define MPU_INT_PIN     				GPIO_PIN_13
#define MPU_INT_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOB_CLK_ENABLE()
#define MPU_INT_EXTI_IRQn				EXTI15_10_IRQn
#define MPU_INT_EXTI_IRQHandler         EXTI15_10_IRQHandler

#define MAG_RDRY_GPIO    				GPIOB
#define MAG_RDRY_PIN     				GPIO_PIN_12
#define MAG_RDRY_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOB_CLK_ENABLE()
#define MAG_RDRY_EXTI_IRQn				EXTI15_10_IRQn
#define MAG_RDRY_EXTI_IRQHandler        EXTI15_10_IRQHandler

#define PWMO12_TIM						TIM1
#define PWMO12_TIM_CLK_ENABLE()			__HAL_RCC_TIM1_CLK_ENABLE()
#define PWMO12_CLK_ENABLE()    			__HAL_RCC_GPIOA_CLK_ENABLE()
#define PWMO12_GPIO    					GPIOA
#define PWMO1_PIN     					GPIO_PIN_8
#define PWMO2_PIN     					GPIO_PIN_11

#define PWMO36_TIM						TIM4
#define PWMO36_TIM_CLK_ENABLE()			__HAL_RCC_TIM4_CLK_ENABLE()
#define PWMO36_CLK_ENABLE()    			__HAL_RCC_GPIOB_CLK_ENABLE()
#define PWMO36_GPIO    					GPIOB
#define PWMO3_PIN     					GPIO_PIN_6
#define PWMO4_PIN     					GPIO_PIN_7
#define PWMO5_PIN     					GPIO_PIN_8
#define PWMO6_PIN     					GPIO_PIN_9

#define PWMI14_TIM						TIM2
#define PWMI14_TIM_CLK_ENABLE()			__HAL_RCC_TIM2_CLK_ENABLE()
#define PWMI14_CLK_ENABLE()    			__HAL_RCC_GPIOA_CLK_ENABLE()
#define PWMI14_TIM_IRQn					TIM2_IRQn
#define PWMI14_TIM_IRQHandler           TIM2_IRQHandler
#define PWMI14_EXTI_IRQn				EXTI0_IRQn
#define PWMI14_EXTI_IRQHandler          EXTI0_IRQHandler
#define PWMI14_GPIO    					GPIOA
#define PWMI1_PIN     					GPIO_PIN_0
#define PWMI2_PIN     					GPIO_PIN_1
#define PWMI3_PIN     					GPIO_PIN_2
#define PWMI4_PIN     					GPIO_PIN_3

#define PWMI1_CCER_CCP					*((__IO uint32_t*)(BITBAND_PERIPH(&(TIM2->CCER),1)))
#define PWMI2_CCER_CCP					*((__IO uint32_t*)(BITBAND_PERIPH(&(TIM2->CCER),5)))
#define PWMI3_CCER_CCP					*((__IO uint32_t*)(BITBAND_PERIPH(&(TIM2->CCER),9)))
#define PWMI4_CCER_CCP					*((__IO uint32_t*)(BITBAND_PERIPH(&(TIM2->CCER),13)))

#define PWMI1_POLARITY_FALLING()		(PWMI1_CCER_CCP=1)
#define PWMI1_POLARITY_RISING()			(PWMI1_CCER_CCP=0)
#define PWMI2_POLARITY_FALLING()		(PWMI2_CCER_CCP=1)
#define PWMI2_POLARITY_RISING()			(PWMI2_CCER_CCP=0)
#define PWMI3_POLARITY_FALLING()		(PWMI3_CCER_CCP=1)
#define PWMI3_POLARITY_RISING()			(PWMI3_CCER_CCP=0)
#define PWMI4_POLARITY_FALLING()		(PWMI4_CCER_CCP=1)
#define PWMI4_POLARITY_RISING()			(PWMI4_CCER_CCP=0)

#define PWMI58_TIM						TIM3
#define PWMI58_TIM_CLK_ENABLE()			__HAL_RCC_TIM3_CLK_ENABLE()
#define PWMI56_CLK_ENABLE()    			__HAL_RCC_GPIOA_CLK_ENABLE()
#define PWMI78_CLK_ENABLE()    			__HAL_RCC_GPIOB_CLK_ENABLE()
#define PWMI56_GPIO    					GPIOA
#define PWMI5_PIN     					GPIO_PIN_6
#define PWMI6_PIN     					GPIO_PIN_7
#define PWMI78_GPIO    					GPIOB
#define PWMI7_PIN     					GPIO_PIN_0
#define PWMI8_PIN     					GPIO_PIN_1

#define VBAT_ADC_GPIO               	GPIOA
#define VBAT_ADC_GPIO_PIN           	GPIO_PIN_4
#define VBAT_ADC_CHANNEL            	ADC_Channel_4
#define VBAT_ADC_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOA_CLK_ENABLE()
#define VBAT_ADC_CLK_ENABLE()    		__HAL_RCC_ADC1_CLK_ENABLE()

/* Definition for USARTx clock resources */
#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE()
#define USARTx_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler

/* Definition for USARTx Pins */
#define USARTx_RX_GPIO     		         GPIOA
#define USARTx_RX_PIN                    GPIO_PIN_10
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_TX_GPIO		              GPIOA
#define USARTx_TX_PIN                    GPIO_PIN_9
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL			DMA1_Channel4
#define USARTx_RX_DMA_CHANNEL			DMA1_Channel5
#define USARTx_DMA_CLK_ENABLE()			__HAL_RCC_DMA1_CLK_ENABLE()

/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn              DMA1_Channel4_IRQn
#define USARTx_DMA_RX_IRQn              DMA1_Channel5_IRQn
#define USARTx_DMA_TX_IRQHandler        DMA1_Channel4_IRQHandler
#define USARTx_DMA_RX_IRQHandler        DMA1_Channel5_IRQHandler

/* Definition for I2Cx clock resources */
#define I2Cx                            I2C2
#define I2Cx_CLK_ENABLE()               __HAL_RCC_I2C2_CLK_ENABLE()
#define I2Cx_FORCE_RESET()              __HAL_RCC_I2C2_FORCE_RESET()
#define I2Cx_RELEASE_RESET()            __HAL_RCC_I2C2_RELEASE_RESET()

#define I2Cx_SPEEDCLOCK   				400000
#define I2Cx_DUTYCYCLE    				I2C_DUTYCYCLE_2

/* Definition for I2Cx Pins */
#define I2Cx_SCL_GPIO					GPIOB
#define I2Cx_SCL_PIN                    GPIO_PIN_10
#define I2Cx_SDA_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define I2Cx_SDA_GPIO					GPIOB
#define I2Cx_SDA_PIN                    GPIO_PIN_11
#define I2Cx_SCL_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

/* Definition for I2Cx's NVIC */
#define I2Cx_EV_IRQn                    I2C2_EV_IRQn
#define I2Cx_ER_IRQn                    I2C2_ER_IRQn
#define I2Cx_EV_IRQHandler              I2C2_EV_IRQHandler
#define I2Cx_ER_IRQHandler              I2C2_ER_IRQHandler



#endif /* PLATFORM_CFG_H_ */
