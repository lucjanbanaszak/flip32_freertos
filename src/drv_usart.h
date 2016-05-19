/*
 * drv_usart.h
 *
 *  Created on: 1 kwi 2016
 *      Author: lucja
 */

#ifndef DRV_USART_H_
#define DRV_USART_H_

typedef void (*usartRXCallback_t)( uint8_t c, portBASE_TYPE* );

HAL_StatusTypeDef usartInit( usartRXCallback_t rxcallback );
extern UART_HandleTypeDef UartHandle;

#endif /* DRV_USART_H_ */
