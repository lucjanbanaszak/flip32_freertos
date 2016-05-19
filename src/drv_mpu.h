/*
 * drv_mpu.h
 *
 *  Created on: 23 mar 2016
 *      Author: solaris
 */

#ifndef DRV_MPU_H_
#define DRV_MPU_H_

#include "FreeRTOS.h"
#include "semphr.h"
#include "sensors.h"

HAL_StatusTypeDef mpuDetect(sensor_t *acc, sensor_t *gyro );

extern SemaphoreHandle_t MPU_DataReady_sem;
extern uint16_t acc_1G;

#endif /* DRV_MPU_H_ */
