/*
 * drv_gpio.h
 *
 *  Created on: 1 kwi 2016
 *      Author: lucja
 */

#ifndef DRV_GPIO_H_
#define DRV_GPIO_H_

HAL_StatusTypeDef gpioInit( void );

extern const PIN_t PIN_LED0;
extern const PIN_t PIN_LED1;
extern const PIN_t PIN_BEEP;
extern const PIN_t PIN_BARO_XCLR;
extern const PIN_t PIN_BARO_EOC;
extern const PIN_t PIN_USARTx_RX;
extern const PIN_t PIN_USARTx_TX;
extern const PIN_t PIN_I2Cx_SCL;
extern const PIN_t PIN_I2Cx_SDA;
extern const PIN_t PIN_MPU_INT;
extern const PIN_t PIN_MAG_RDRY;
extern const PIN_t PIN_PWMO1;
extern const PIN_t PIN_PWMO2;
extern const PIN_t PIN_PWMO3;
extern const PIN_t PIN_PWMO4;
extern const PIN_t PIN_PWMO5;
extern const PIN_t PIN_PWMO6;
extern const PIN_t PIN_PWMI1;
extern const PIN_t PIN_PWMI2;
extern const PIN_t PIN_PWMI3;
extern const PIN_t PIN_PWMI4;
extern const PIN_t PIN_PWMI5;
extern const PIN_t PIN_PWMI6;
extern const PIN_t PIN_PWMI7;
extern const PIN_t PIN_PWMI8;

#endif /* DRV_GPIO_H_ */
