/*
 * drv_pwm.c
 *
 *  Created on: 1 kwi 2016
 *      Author: lucja
 */
#include "platform_cfg.h"

TIM_HandleTypeDef Timer1Handle;
TIM_HandleTypeDef Timer2Handle;
TIM_HandleTypeDef Timer4Handle;

__IO int32_t pwmi1_val = 1500;
__IO int32_t pwmi2_val = 1500;
__IO int32_t pwmi3_val = 1500;
__IO int32_t pwmi4_val = 1500;

HAL_StatusTypeDef pwmInit( void )
{
	/************************************************************/
	/** PPM Input ***********************************************/
	/************************************************************/
	TIM_IC_InitTypeDef sConfigIC;

	PWMI14_CLK_ENABLE();
	__HAL_RCC_AFIO_CLK_ENABLE();

	HAL_GPIO_Init( PIN_PWMI1.gpio, (GPIO_InitTypeDef*)&PIN_PWMI1.cfg );
	HAL_GPIO_Init( PIN_PWMI2.gpio, (GPIO_InitTypeDef*)&PIN_PWMI2.cfg );
	HAL_GPIO_Init( PIN_PWMI3.gpio, (GPIO_InitTypeDef*)&PIN_PWMI3.cfg );
	HAL_GPIO_Init( PIN_PWMI4.gpio, (GPIO_InitTypeDef*)&PIN_PWMI4.cfg );

	/* NVIC for TIM2 */
	HAL_NVIC_SetPriority(PWMI14_TIM_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+2, 0);
	HAL_NVIC_EnableIRQ(PWMI14_TIM_IRQn);

	PWMI14_TIM_CLK_ENABLE();

	Timer2Handle.Instance = TIM2;
	Timer2Handle.Init.Prescaler = (uint32_t)(SystemCoreClock / 1000000) - 1;
	Timer2Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Timer2Handle.Init.Period = 0xffff;
	Timer2Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_IC_Init(&Timer2Handle);

	sConfigIC.ICPolarity = TIM_ICPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;

	HAL_TIM_IC_ConfigChannel(&Timer2Handle, &sConfigIC, TIM_CHANNEL_1);
	HAL_TIM_IC_ConfigChannel(&Timer2Handle, &sConfigIC, TIM_CHANNEL_2);
	HAL_TIM_IC_ConfigChannel(&Timer2Handle, &sConfigIC, TIM_CHANNEL_3);
	HAL_TIM_IC_ConfigChannel(&Timer2Handle, &sConfigIC, TIM_CHANNEL_4);

	HAL_TIM_IC_Start_IT(&Timer2Handle, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&Timer2Handle, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&Timer2Handle, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&Timer2Handle, TIM_CHANNEL_4);

	/************************************************************/
	/** PPM Output **********************************************/
	/************************************************************/
	TIM_OC_InitTypeDef sConfig;

	PWMO12_CLK_ENABLE();
	PWMO36_CLK_ENABLE();

	HAL_GPIO_Init( PIN_PWMO1.gpio, (GPIO_InitTypeDef*)&PIN_PWMO1.cfg );
	HAL_GPIO_Init( PIN_PWMO2.gpio, (GPIO_InitTypeDef*)&PIN_PWMO2.cfg );
	HAL_GPIO_Init( PIN_PWMO3.gpio, (GPIO_InitTypeDef*)&PIN_PWMO3.cfg );
	HAL_GPIO_Init( PIN_PWMO4.gpio, (GPIO_InitTypeDef*)&PIN_PWMO4.cfg );
	HAL_GPIO_Init( PIN_PWMO5.gpio, (GPIO_InitTypeDef*)&PIN_PWMO5.cfg );
	HAL_GPIO_Init( PIN_PWMO6.gpio, (GPIO_InitTypeDef*)&PIN_PWMO6.cfg );

	PWMO12_TIM_CLK_ENABLE();
	PWMO36_TIM_CLK_ENABLE();

	Timer1Handle.Instance = PWMO12_TIM;
	Timer1Handle.Init.Prescaler = (uint32_t)(SystemCoreClock / 1000000) - 1;
	Timer1Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Timer1Handle.Init.Period = 15000;
	Timer1Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	Timer1Handle.Init.RepetitionCounter = 0;
	HAL_TIM_PWM_Init(&Timer1Handle);

	Timer4Handle.Instance = PWMO36_TIM;
	Timer4Handle.Init.Prescaler = (uint32_t)(SystemCoreClock / 1000000) - 1;
	Timer4Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Timer4Handle.Init.Period = 1000;
	Timer4Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	Timer4Handle.Init.RepetitionCounter = 0;
	HAL_TIM_PWM_Init(&Timer4Handle);

	sConfig.OCMode = TIM_OCMODE_PWM1;
	sConfig.Pulse = 0;
	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfig.OCFastMode = TIM_OCFAST_DISABLE;
	sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	HAL_TIM_PWM_ConfigChannel(&Timer1Handle, &sConfig, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&Timer1Handle, &sConfig, TIM_CHANNEL_4);

//	HAL_TIM_PWM_ConfigChannel(&Timer4Handle, &sConfig, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&Timer4Handle, &sConfig, TIM_CHANNEL_2);
//	HAL_TIM_PWM_ConfigChannel(&Timer4Handle, &sConfig, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&Timer4Handle, &sConfig, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&Timer1Handle, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&Timer1Handle, TIM_CHANNEL_4);

//	HAL_TIM_PWM_Start(&Timer4Handle, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&Timer4Handle, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&Timer4Handle, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&Timer4Handle, TIM_CHANNEL_4);

	return HAL_OK;
}

void PWMI14_TIM_IRQHandler( void )
{
	static uint32_t ic1_val = 0;
	static uint32_t ic2_val = 0;
	static uint32_t ic3_val = 0;
	static uint32_t ic4_val = 0;

    if( __HAL_TIM_GET_FLAG(&Timer2Handle, TIM_FLAG_CC1) )
	{
    	uint32_t ic_val = Timer2Handle.Instance->CCR1;
    	if( PWMI1_CCER_CCP )	pwmi1_val = (ic_val >= ic1_val) ? (ic_val-ic1_val):((0xffff-ic1_val)+ic_val);
    	else					ic1_val = ic_val;

    	PWMI1_CCER_CCP ^= 1;
	};

    if( __HAL_TIM_GET_FLAG(&Timer2Handle, TIM_FLAG_CC2) )
 	{
    	uint32_t ic_val = Timer2Handle.Instance->CCR2;
     	if( PWMI2_CCER_CCP )	pwmi2_val = (ic_val >= ic2_val) ? (ic_val-ic2_val):((0xffff-ic2_val)+ic_val);
     	else					ic2_val = ic_val;

     	PWMI2_CCER_CCP ^= 1;
 	};

    if( __HAL_TIM_GET_FLAG(&Timer2Handle, TIM_FLAG_CC3) )
 	{
    	uint32_t ic_val = Timer2Handle.Instance->CCR3;
     	if( PWMI3_CCER_CCP )	pwmi3_val = (ic_val >= ic3_val) ? (ic_val-ic3_val):((0xffff-ic3_val)+ic_val);
     	else					ic3_val = ic_val;

     	PWMI3_CCER_CCP ^= 1;
 	};

    if( __HAL_TIM_GET_FLAG(&Timer2Handle, TIM_FLAG_CC4) )
  	{
    	uint32_t ic_val = Timer2Handle.Instance->CCR4;
      	if( PWMI4_CCER_CCP )	pwmi4_val = (ic_val >= ic4_val) ? (ic_val-ic4_val):((0xffff-ic4_val)+ic_val);
      	else					ic4_val = ic_val;

      	PWMI4_CCER_CCP ^= 1;
  	};
};

