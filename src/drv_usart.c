/*
 * drv_usart.c
 *
 *  Created on: 1 kwi 2016
 *      Author: lucja
 */
#include "platform_cfg.h"
#include "drv_usart.h"
#include "drv_msp.h"
#include "printf.h"

UART_HandleTypeDef UartHandle;

usartRXCallback_t usartRXCallback = NULL;

void _putc(void *UartHandle, char c)
{
    HAL_UART_Transmit(UartHandle, (uint8_t *)&c, 1, 0xFFFF);
};

HAL_StatusTypeDef usartInit( usartRXCallback_t rxcallback )
{
	/* USART */
	USARTx_CLK_ENABLE();
	USARTx_RX_GPIO_CLK_ENABLE();
	USARTx_TX_GPIO_CLK_ENABLE();
	__HAL_RCC_AFIO_CLK_ENABLE();

	HAL_GPIO_Init( PIN_USARTx_RX.gpio, (GPIO_InitTypeDef*)&PIN_USARTx_RX.cfg );
	HAL_GPIO_Init( PIN_USARTx_TX.gpio, (GPIO_InitTypeDef*)&PIN_USARTx_TX.cfg );

	UartHandle.Instance        = USARTx;
	UartHandle.Init.BaudRate   = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits   = UART_STOPBITS_1;
	UartHandle.Init.Parity     = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;

	if(HAL_UART_Init(&UartHandle) != HAL_OK) return HAL_ERROR;

	init_printf( &UartHandle, _putc );

	if( rxcallback == NULL ) return HAL_ERROR;

	usartRXCallback = rxcallback;

	/* NVIC for USART */
	HAL_NVIC_SetPriority(USARTx_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1, 0);
	HAL_NVIC_EnableIRQ(USARTx_IRQn);

     /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_RXNE);

	printf("\x1b[2J");
	printf("\x1b[2J");

	return HAL_OK;
};

void USARTx_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken;

	uint32_t tmp_flag = __HAL_UART_GET_FLAG( &UartHandle, UART_FLAG_RXNE);
	uint32_t tmp_it_source = __HAL_UART_GET_IT_SOURCE( &UartHandle, UART_IT_RXNE);

    /* UART in mode Receiver ---------------------------------------------------*/
    if((tmp_flag != RESET) && (tmp_it_source != RESET))
    {
    	uint8_t c = UartHandle.Instance->DR;
    	usartRXCallback( c, &xHigherPriorityTaskWoken );
    };

	/* Switch tasks if necessary. */
	if( xHigherPriorityTaskWoken != pdFALSE ) portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
