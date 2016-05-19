/*
 * drv_mag.h
 *
 *  Created on: 30 mar 2016
 *      Author: solaris
 */

#ifndef DRV_MAG_H_
#define DRV_MAG_H_

HAL_StatusTypeDef magDetect(sensor_t *mag);
void MAG_RDRY_EXTI_Callback(void);

extern SemaphoreHandle_t MAG_DataReady_sem;

#endif /* DRV_MAG_H_ */
