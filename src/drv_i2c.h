/*
 * drv_i2c.h
 *
 *  Created on: 25 mar 2016
 *      Author: solaris
 */

#ifndef DRV_I2C_H_
#define DRV_I2C_H_

#include "semphr.h"

extern I2C_HandleTypeDef I2cHandle;
extern SemaphoreHandle_t I2C_MemTxCplt_sem;
extern SemaphoreHandle_t I2C_MemRxCplt_sem;

HAL_StatusTypeDef i2cInit( void );
HAL_StatusTypeDef i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t *buff, uint8_t len );
HAL_StatusTypeDef i2cWrite(uint8_t addr, uint8_t reg, uint8_t data);
HAL_StatusTypeDef i2cRead(uint8_t addr, uint8_t reg, uint8_t *buff, uint8_t len );

#endif /* DRV_I2C_H_ */
