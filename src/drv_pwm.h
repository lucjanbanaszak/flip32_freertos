/*
 * drv_pwm.h
 *
 *  Created on: 1 kwi 2016
 *      Author: lucja
 */

#ifndef DRV_PWM_H_
#define DRV_PWM_H_

HAL_StatusTypeDef pwmInit( void );

extern TIM_HandleTypeDef Timer1Handle;
extern TIM_HandleTypeDef Timer2Handle;
extern TIM_HandleTypeDef Timer4Handle;

extern __IO int32_t pwmi1_val;
extern __IO int32_t pwmi2_val;
extern __IO int32_t pwmi3_val;
extern __IO int32_t pwmi4_val;

#endif /* DRV_PWM_H_ */
