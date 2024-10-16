/*
 * stm32l4xx_USART_driver.h
 *
 *  Created on: 14 Oct 2024
 *      Author: Pillow sleeper
 */

#ifndef STM32_L476R_DRIVERS_INC_STM32L4XX_USART_DRIVER_H_
#define STM32_L476R_DRIVERS_INC_STM32L4XX_USART_DRIVER_H_

#include "stm32l4xx.h"

// Configuration strucutre for USARTx peripheral
typedef struct{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;

// Handle structure for USARTx peripheral
typedef struct{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
};

// Register defintion for USARTx peripheral
typedef struct{
	uint32_t CR1;
	uint32_t CR2;
	uint32_t CR3;
	uint32_t BRR;
	uint32_t GTPR;
	uint32_t RTOR;
	uint32_t RQR;
	uint32_t ISR;
	uint32_t ICR;
	uint32_t RDR;
	uint32_t TDR;
}USART_RegDef_t;


#endif /* STM32_L476R_DRIVERS_INC_STM32L4XX_USART_DRIVER_H_ */
