/*
 * stm32l4xx_USART_driver.c
 *
 *  Created on: 14 Oct 2024
 *      Author: Pillow sleeper
 */


#include "stm32l4xx_USART_driver.h"

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		if(pUSARTx == USART1){
			USART1_PCLK_EN();
		}else if(pUSARTx == USART2){
			USART2_PCLK_EN();
		}else if(pUSARTx == USART3){
			USART3_PCLK_EN();
		}else if(pUSARTx == UART4){
			UART4_PCLK_EN();
		}else if(pUSARTx == UART5){
			UART5_PCLK_EN();
		}
	}else{
		if(pUSARTx == USART1){
			USART1_PCLK_DI();
		}else if(pUSARTx == USART2){
			USART2_PCLK_DI();
		}else if(pUSARTx == USART3){
			USART3_PCLK_DI();
		}else if(pUSARTx == UART4){
			UART4_PCLK_DI();
		}else if(pUSARTx == UART5){
			UART5_PCLK_DI();
		}
	}
}


void USART_USARTControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pUSARTx->CR1 |= (1 << USARTx_CR1_UE);
	}else{
		pUSARTx->CR1 &= ~(1 << USARTx_CR1_UE);
	}
}


uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName){
	if(pUSARTx->ISR & FlagName){
		return FLAG_SET;
	}

	return FLAG_RESET;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t FlagName){
	pUSARTx->ICR |= ENABLE << FlagName;
}

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi){
    if(EnOrDi == ENABLE)
    {
        if(IRQNumber <= 31)
        {
            /* Program ISER0 register */
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64)
        {
            /* Program ISER1 register (32 to 63) */
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            /* Program ISER2 register (64 to 95) */
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    }
    else
    {
        if(IRQNumber <= 31)
        {
            /* Program ICER0 register */
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64)
        {
            /* Program ICER1 register (32 to 63) */
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            /* Program ICER2 register (64 to 95) */
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    }
}

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

void USART_Init(USART_Handle_t *pUSARTHandle){
	// Temporary variable
	uint32_t tempreg = 0;

	/********************** Configure CR1 Register **********************/

	// Intialize the clock to the given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	// Enable USART Tx and Rx enginees according to the USART Mode configuration
	// The mode macros are set such that the correct values will be placed in the RE and TE registers
	tempreg |= (pUSARTHandle->USART_Config.USART_Mode << USARTx_CR1_RE);

	// Configure the word length register
	// If eight bit word length is selected both M0 and M1 stay empty
	if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_7BITS){
		tempreg |= (ENABLE << USARTx_CR1_M1);
	}else if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
		tempreg |= (ENABLE << USARTx_CR1_M0);
	}

	// Configure the parity control registers
	if(pUSARTHandle->USART_Config.USART_ParityControl != USART_PARITY_DISABLE){
		tempreg |= (ENABLE << USARTx_CR1_PCE);
		tempreg |= (pUSARTHandle->USART_Config.USART_ParityControl << USARTx_CR1_PS);
	}

	pUSARTHandle->pUSARTx->CR1 |= tempreg;
	tempreg = 0;

	/********************** Configure CR2 Register **********************/

	// Configure number of stop bits during a USART frame transmission
	tempreg |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USARTx_CR2_STOP);

	pUSARTHandle->pUSARTx->CR2 |= tempreg;
	tempreg = 0;
	/********************** Configure CR3 Register **********************/

	// Configure USART hardware flow control
	// The hardware control macros are set such that the correct values will be placed in the RTSE and CTSE registers
	tempreg |= (pUSARTHandle->USART_Config.USART_HWFlowContro << USARTx_CR3_RTSE);

	pUSARTHandle->pUSARTx->CR3 |= tempreg;

	/********************** Configure BRR Register **********************/


}
