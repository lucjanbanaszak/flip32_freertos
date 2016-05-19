/*
 * common.h
 *
 *  Created on: 30 mar 2016
 *      Author: solaris
 */

#ifndef COMMON_H_
#define COMMON_H_

#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "stdbool.h"



typedef struct{
	GPIO_TypeDef *gpio;
	GPIO_InitTypeDef cfg;
} PIN_t;

#define PIN_State(A)		(A.gpio->IDR&A.cfg.Pin)
#define PIN_Set(A)			A.gpio->BSRR = A.cfg.Pin
#define PIN_Clear(A)		A.gpio->BRR = A.cfg.Pin
#define PIN_Toggle(A)		A.gpio->ODR ^= A.cfg.Pin

#define LEDOn(A)			PIN_Clear(A)
#define LEDOff(A)			PIN_Set(A)
#define LEDToggle(A)		PIN_Toggle(A)

#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif /* M_PI */

#define RADX10 (M_PI / 1800.0f)                  // 0.001745329252f
#define RAD    (M_PI / 180.0f)

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* bitband type */
typedef volatile uint32_t * const bitband_t;

/* base address for bit banding */
#define BITBAND_SRAM_REF               		(0x20000000)
/* base address for bit banding */
#define BITBAND_SRAM_BASE              		(0x22000000)
/* base address for bit banding */
#define BITBAND_PERIPH_REF               	(0x40000000)
/* base address for bit banding */
#define BITBAND_PERIPH_BASE              	(0x42000000)

/* sram bit band */
#define BITBAND_SRAM(address, bit)     ((void*)(BITBAND_SRAM_BASE +   \
		(((uint32_t)address) - BITBAND_SRAM_REF) * 32 + (bit) * 4))

/* periph bit band */
#define BITBAND_PERIPH(address, bit)   ((void *)(BITBAND_PERIPH_BASE + \
		(((uint32_t)address) - BITBAND_PERIPH_REF) * 32 + (bit) * 4))

// Chip Unique ID on F103
#define U_ID_0 (*(uint32_t*)0x1FFFF7E8)
#define U_ID_1 (*(uint32_t*)0x1FFFF7EC)
#define U_ID_2 (*(uint32_t*)0x1FFFF7F0)

typedef GPIO_InitTypeDef gpio_config_t;

#ifndef __CC_ARM
#define REQUIRE_CC_ARM_PRINTF_SUPPORT
#define REQUIRE_PRINTF_LONG_SUPPORT
#endif

#endif /* COMMON_H_ */
