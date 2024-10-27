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

	// Write the TE and RE bits in CR1
	tempreg |= (pUSARTHandle->USART_Config.USART_Mode << USARTx_CR1_RE);

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
	tempreg |= (pUSARTHandle->USART_Config.USART_HWFlowControl << USARTx_CR3_RTSE);

	pUSARTHandle->pUSARTx->CR3 |= tempreg;

	// Configure DMA use for Transmitter and Receiver
	tempreg |= (pUSARTHandle->USART_Config.USART_DMATransmitter << USARTx_CR3_DMAT);
	tempreg |= (pUSARTHandle->USART_Config.USART_DMAReceiver << USARTx_CR3_DMAR);

	/********************** Configure BRR Register **********************/
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}


void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len){
	uint16_t *pdata;
	pdata = (uint16_t*)pTxBuffer;


	// Loop over until "Len" number of bytes have been transferred
	for(uint32_t i = 0; i < Len; i++){
		// Wait until the TXE flag is set in the ISR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USARTx_TXE_FLAG));

		// Check if the USART wordlength is 7, 8 or 9 bits
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_7BITS){
			// Load the data into the Transfer Data Register (mask the last bit as 0)
			pUSARTHandle->pUSARTx->TDR = (*pdata & (uint16_t)0xFE);

			pTxBuffer++;
		}else if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS){
			// Load the data into the Transfer Data Register
			pUSARTHandle->pUSARTx->TDR = (*pdata & (uint16_t)0xFF);

			pTxBuffer++;
		}else{
			// If 9 bit wordlength load the DR with a 2 byte masking that is 0 for any except the first 9 bits

			// Load the data into the Transfer Data Register
			pUSARTHandle->pUSARTx->TDR = (*pdata & (uint16_t)0x01FF);

			pTxBuffer++;

			// If the parity check is not disabled 9 bits of user data will be sent therefore the TxBuffer must
			// be incremented twice. Otherwise only 8 bits will be sent and the buffer incremented once (done above)
			// as the 9th bit will be the parity bit
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
				pTxBuffer++;
			}

		}
	}
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){

	// Loop over until "Len" number of bytes have been transferred
	// Incrementing of RxBuffer done as the last instruction
	for(uint32_t i = 0; i < Len; i++){
		// Wait until the RXNE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USARTx_RXNE_FLAG));

		// Check if the USART wordlength is 7, 8 or 9 bits
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_7BITS){

			// Check the parity bit setting
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
				// We are receiving 7 bits of data

				// Read the first 7 bits of data. Mask: 0x7F
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->RDR & (uint16_t)0x7F);
			}else{
				// We are receiving 6 bits of data and a parity bit

				// Read the first 6 bits of data. Mask: 0x3F
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->RDR & (uint16_t)0x3F);

			}

		}else if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS){

			// Check the parity bit setting
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
				// We are receiving 8 bits of data

				// Read the first 8 bits of data. Mask: 0xFF
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->RDR & (uint16_t)0xFF);


			}else{
				// We are receiving 7 bits of data and a parity bit

				// Read the first 7 bits of data. Mask: 0x7F
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->RDR & (uint16_t)0x7F);

			}

		}else{

			// Check the parity bit setting
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
				// We are receiving 9 bits of data

				// Read the first 9 bits of data. Mask: 0x01FF
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->RDR & (uint16_t)0x01FF);

				// The RxBuffer has to be incremented twice as more than one byte of data is received
				// Second increment done beneath the if else chain
				pRxBuffer++;

			}else{
				// We are receiving 8 bits of data and a parity bit

				// Read the first 8 bits of data. Mask: 0xFF
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->RDR & (uint16_t)0xFF);

			}
		}

		// Increment the RxBuffer
		pRxBuffer++;
	}
}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len){
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate == USART_NOT_BUSY_IN_TX){
		pUSARTHandle->TransmitLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		// Enable the interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (ENABLE << USARTx_CR1_TXEIE);

		// Enable the interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (ENABLE << USARTx_CR1_TCIE);
	}

	return txstate;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
	uint8_t rxstate = pUSARTHandle->TxBusyState;

	if(rxstate == USART_NOT_BUSY_IN_RX){
		pUSARTHandle->TransmitLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		// Enable the interrupt for RXE
		pUSARTHandle->pUSARTx->CR1 |= (ENABLE << USARTx_CR1_RXNEIE);

	}

	return rxstate;
}

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate){

	// Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	// Varaiables to hold Mantissa and Fraction values
	uint32_t M_part, F_part;

	uint32_t tempreg = 0;

	// Get the value of APB bus clock and store it in the PCLKx variable
	if(pUSARTx == USART1){
		// Get APB2 clock
		PCLKx = RCC_GetPCLK2Value();
	}else{
		// Get APB1 clock
		PCLKx = RCC_GetPCLK1Value();
	}

	// Check for the OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USARTx_CR1_OVER8)){
		// OVER8 enabled, sampling by 8

		// The 25 come from the fact that we multiply the original (f/(8*BR)) equation by 100 to
		// turn any decimal points into whole numbers so they can later be broken in mantissa and
		// fraction parts
		usartdiv = ((25 * PCLKx) / (2 * BaudRate));

	}else{
		// OVER8 disable, sampling by 16
		// The 25 come from the fact that we multiply the original (f/(16*BR)) equation by 100 to
		// turn any decimal points into whole numbers so they can later be broken in mantissa and
		// fraction parts
		usartdiv = ((25 * PCLKx) / (4 * BaudRate));
	}

	// Calculate the mantissa and first fractional part
	M_part = usartdiv/100;
	F_part = usartdiv - (M_part * 100);

	// Calculate the final fractional
	if(pUSARTx->CR1 & (1 << USARTx_CR1_OVER8)){
		// OVER8 enabled, sampling by 8

		// +50 so that the extracted number is rounded to the correct place (extracting an integer
		// from a float alway rounds down in C, so we do the rounding ourselves)
		F_part = (((F_part * 8) + 50)/100) &((uint8_t) 0x07);

		// For a sampling of 8 the fractional value must be bit-shifted to the left 1
		F_part = F_part << 1;
	}else{
		// OVER8 disanble, sampling by 16
		F_part = (((F_part * 16) + 50)/100) &((uint8_t) 0x0F);
	}

	// Shift the mantissa to the correct part of the BRR
	tempreg |= (M_part << 4);
	tempreg |= F_part;

	pUSARTx->BRR |= tempreg;
}


