/*
 * drv_i2c.c
 *
 *  Created on: 25 mar 2016
 *      Author: solaris
 */

#include "platform_cfg.h"
#include "drv_i2c.h"

I2C_HandleTypeDef I2cHandle;
SemaphoreHandle_t I2C_Lock_sem;
SemaphoreHandle_t I2C_MemTxCplt_sem;
SemaphoreHandle_t I2C_MemRxCplt_sem;

HAL_StatusTypeDef i2cInit( void )
{
	/* create binary semaphore */
	I2C_Lock_sem = xSemaphoreCreateBinary();
	I2C_MemTxCplt_sem = xSemaphoreCreateBinary();
	I2C_MemRxCplt_sem = xSemaphoreCreateBinary();

	xSemaphoreTake( I2C_MemTxCplt_sem, 0);
	xSemaphoreTake( I2C_MemRxCplt_sem, 0);
	xSemaphoreGive( I2C_Lock_sem );

	I2Cx_CLK_ENABLE();
	I2Cx_SCL_GPIO_CLK_ENABLE();
	I2Cx_SDA_GPIO_CLK_ENABLE();
	__HAL_RCC_AFIO_CLK_ENABLE();

	HAL_GPIO_Init( PIN_I2Cx_SCL.gpio, (GPIO_InitTypeDef*)&PIN_I2Cx_SCL.cfg );
	HAL_GPIO_Init( PIN_I2Cx_SDA.gpio, (GPIO_InitTypeDef*)&PIN_I2Cx_SDA.cfg );

	/* NVIC for I2Cx */
	HAL_NVIC_SetPriority(I2Cx_EV_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
	HAL_NVIC_EnableIRQ(I2Cx_EV_IRQn);

	HAL_NVIC_SetPriority(I2Cx_ER_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
	HAL_NVIC_EnableIRQ(I2Cx_ER_IRQn);

	/*##-1- Configure the I2C peripheral ######################################*/
	I2cHandle.Instance             = I2Cx;
	I2cHandle.Init.ClockSpeed      = I2Cx_SPEEDCLOCK;
	I2cHandle.Init.DutyCycle       = I2Cx_DUTYCYCLE;
	I2cHandle.Init.OwnAddress1     = 0x00;
	I2cHandle.Init.OwnAddress2	   = 0x00;
	I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

	return HAL_I2C_Init(&I2cHandle);
};

HAL_StatusTypeDef i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t *buff, uint8_t len )
{
	if( xSemaphoreTake( I2C_Lock_sem, 10) != pdPASS ) return HAL_TIMEOUT;

	HAL_StatusTypeDef status = HAL_I2C_Mem_Write_IT(&I2cHandle, (uint16_t)addr, (uint16_t)reg, (uint16_t)I2C_MEMADD_SIZE_8BIT, (uint8_t *)buff, len );
	if( xSemaphoreTake( I2C_MemTxCplt_sem, 10 ) != pdPASS ) return HAL_TIMEOUT;

	xSemaphoreGive( I2C_Lock_sem );
	return status;
};

HAL_StatusTypeDef i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
{
    return i2cWriteBuffer(addr, reg, (uint8_t*)&data, 1);
};

HAL_StatusTypeDef i2cRead(uint8_t addr, uint8_t reg, uint8_t *buff, uint8_t len )
{
	if( xSemaphoreTake( I2C_Lock_sem, 10) != pdPASS ) return HAL_TIMEOUT;

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read_IT(&I2cHandle, (uint16_t)addr, (uint16_t)reg, (uint16_t)I2C_MEMADD_SIZE_8BIT, (uint8_t *)buff, len );
	if( xSemaphoreTake( I2C_MemRxCplt_sem, 10 ) != pdPASS ) return HAL_TIMEOUT;

	xSemaphoreGive( I2C_Lock_sem );
	return status;
};

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR( I2C_MemTxCplt_sem, &xHigherPriorityTaskWoken );

	if( xHigherPriorityTaskWoken != pdFALSE ) portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
};

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR( I2C_MemRxCplt_sem, &xHigherPriorityTaskWoken );

	if( xHigherPriorityTaskWoken != pdFALSE ) portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
};

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
};

void I2Cx_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&I2cHandle);
}

void I2Cx_ER_IRQHandler(void)
{
  HAL_I2C_ER_IRQHandler(&I2cHandle);
}
