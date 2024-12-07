/*
 * stm32l4xx_USART_driver.c
 *
 *  Created on: 14 Oct 2024
 *      Author: Pillow sleeper
 */

#include <stdbool.h>
#include "stm32l4xx_USART_driver.h"

static void USART_LoadTDR(USART_Handle_t *pUSARTHandle);
static void USART_IRQHandler(USART_Handle_t *pUSARTHandle);
static bool USART_SetHandleLink(USART_Handle_t *pUSARTHandle);

USART_Handle_t *pUSART1HandleLink;
USART_Handle_t *pUSART2HandleLink;
USART_Handle_t *pUSART3HandleLink;
USART_Handle_t *pUART4HandleLink;
USART_Handle_t *pUART5HandleLink;

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

void USART_IRQInterruptControl(USART_Handle_t *pUSARTHandle, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		// Do not turn on the TXE or TC bits as they will DOS the interrupt handler if nothing is being sent
		pUSARTHandle->pUSARTx->CR1 |= (0x13 << USARTx_CR1_IDLEIE);
		pUSARTHandle->pUSARTx->CR3 |= (0x01 << USARTx_CR3_EIE);
	}else{
		pUSARTHandle->pUSARTx->CR1 &= ~(0x13 << USARTx_CR1_IDLEIE);
		pUSARTHandle->pUSARTx->CR3 &= ~(0x01 << USARTx_CR3_EIE);

	}
}


uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName){
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

void USART1_IRQHandler (void){
	USART_IRQHandler(pUSART1HandleLink);
}

void USART2_EV_IRQHandler(void){
	USART_IRQHandler(pUSART2HandleLink);
}

void USART3_IRQHandler(void){
	USART_IRQHandler(pUSART3HandleLink);
}

void UART4_IRQHandler(void){
	USART_IRQHandler(pUART4HandleLink);
}

void UART5_IRQHandler(void){
	USART_IRQHandler(pUART5HandleLink);
}

static void USART_IRQHandler(USART_Handle_t *pUSARTHandle){
	// To check whether it is a specific flag that creates an interrupt we have to check
	// both the event flag and the control bit
	uint8_t event_flag, control_bit;

	event_flag = (pUSARTHandle->pUSARTx->ISR >> USARTx_ISR_TXE) & 1;
	control_bit = (pUSARTHandle->pUSARTx->CR1 >> USARTx_CR1_TXEIE) & 1;
	uint8_t teack = (pUSARTHandle->pUSARTx->ISR >> USARTx_ISR_TEACK) & 1;

	// Handle Interrupt generated by TXE event
	if(event_flag && control_bit && teack){
		// The transmit buffer has been transferred to the shift register

		// If there is data left to be sent load it into the TDR register
		if(pUSARTHandle->TransmitLen != pUSARTHandle->nTransmitted){
			USART_LoadTDR(pUSARTHandle);
		}
	}

	event_flag = (pUSARTHandle->pUSARTx->ISR >> USARTx_ISR_CTSIF) & 1;
	control_bit = (pUSARTHandle->pUSARTx->CR3 >> USARTx_CR3_CTSIE) & 1;

	if(event_flag && control_bit){
		// Make the program aware of the interript
		USART_ApplicationEventCallback(pUSARTHandle, USART_CTS_INTERRUPT);

		// Clear the CTS interrupt
		pUSARTHandle->pUSARTx->ICR |= (ENABLE << USARTx_ICR_CTSCF);
	}

	event_flag = (pUSARTHandle->pUSARTx->ISR >> USARTx_ISR_TC) & 1;
	control_bit = (pUSARTHandle->pUSARTx->CR1 >> USARTx_CR1_TCIE) & 1;

	// Handle Interrupt generated by TC event
	if(event_flag && control_bit){
		// A frame of data has been sent

		// If there is still data left to send, or the peripheral is not transmitting, clear the TC bit
		// Otherwise the transmission is complete and the application should be informed
		if(pUSARTHandle->TransmitLen != pUSARTHandle->nTransmitted){
			pUSARTHandle->pUSARTx->ICR |= (ENABLE << USARTx_ICR_TCCF);
		}else{
			USART_ApplicationEventCallback(pUSARTHandle, USART_TX_COMPLETE);

			// Turn off the TCIE and TXR Interrupts
			pUSARTHandle->pUSARTx->CR1 &= ~(1 << USARTx_CR1_TCIE);
			pUSARTHandle->pUSARTx->CR1 &= ~(1 << USARTx_CR1_TXEIE);

			// Turn off transmit mode
			pUSARTHandle->pUSARTx->CR1 &= ~(USART_MODE_ONLY_TX << USARTx_CR1_RE);
		}
	}

	event_flag = (pUSARTHandle->pUSARTx->ISR >> USARTx_ISR_RXNE) & 1;
	control_bit = (pUSARTHandle->pUSARTx->CR1 >> USARTx_CR1_RXNEIE) & 1;

	if(event_flag && control_bit){
		// TODO: Implement receiver logic
	}

	event_flag = (pUSARTHandle->pUSARTx->ISR >> USARTx_ISR_ORE) & 1;
	// control_bit stays the same

	if(event_flag && control_bit){
		USART_ApplicationEventCallback(pUSARTHandle, USART_OVERRUN_ERROR);
	}

	event_flag = (pUSARTHandle->pUSARTx->ISR >> USARTx_ISR_IDLE) & 1;
	control_bit = (pUSARTHandle->pUSARTx->CR1 << USARTx_CR1_IDLEIE) & 1;

	if(event_flag && control_bit){
		USART_ApplicationEventCallback(pUSARTHandle, USART_IDLE_LINE);
	}

	event_flag = (pUSARTHandle->pUSARTx->ISR >> USARTx_ISR_PE);
	control_bit = (pUSARTHandle->pUSARTx->CR1 >> USARTx_CR1_PEIE);

	if(event_flag && control_bit){
		USART_ApplicationEventCallback(pUSARTHandle, USART_PARITY_ERROR);
	}

	event_flag = (pUSARTHandle->pUSARTx->ISR >> USARTx_ISR_LBDF);
	control_bit = (pUSARTHandle->pUSARTx->CR2 >> USARTx_CR2_LBDIE);

	if(event_flag && control_bit){
		USART_ApplicationEventCallback(pUSARTHandle, USART_LIN_BREAK);
	}

	event_flag = (pUSARTHandle->pUSARTx->ISR >> USARTx_ISR_NF);
	control_bit = (pUSARTHandle->pUSARTx->CR3 >> USARTx_CR3_EIE);

	if(event_flag && control_bit){
		USART_ApplicationEventCallback(pUSARTHandle, USART_NOISE_FLAG);
	}

	event_flag = (pUSARTHandle->pUSARTx->ISR >> USARTx_ISR_FE);
	// control_bit stays the same

	if(event_flag && control_bit){
		USART_ApplicationEventCallback(pUSARTHandle, USART_FRAMING_ERROR);
	}

	event_flag = (pUSARTHandle->pUSARTx->ISR >> USARTx_ISR_CMF);
	control_bit = (pUSARTHandle->pUSARTx->CR1 >> USARTx_CR1_CMIE);

	if(event_flag && control_bit){
		USART_ApplicationEventCallback(pUSARTHandle, USART_CHARACTER_MATCH);
	}

	event_flag = (pUSARTHandle->pUSARTx->ISR >> USARTx_ISR_RTOF);
	control_bit = (pUSARTHandle->pUSARTx->CR1 >> USARTx_CR1_RTOIE);

	if(event_flag && control_bit){
		USART_ApplicationEventCallback(pUSARTHandle, USART_RECEIVER_TIMEOUT);
	}

	event_flag = (pUSARTHandle->pUSARTx->ISR >> USARTx_ISR_EOBF);
	control_bit = (pUSARTHandle->pUSARTx->CR1 >> USARTx_CR1_EOBIE);

	if(event_flag && control_bit){
		USART_ApplicationEventCallback(pUSARTHandle, USART_END_OF_BLOCK);
	}

	event_flag = (pUSARTHandle->pUSARTx->ISR >> USARTx_ISR_WUF);
	control_bit = (pUSARTHandle->pUSARTx->CR3 >> USARTx_CR3_WUFIE);

	if(event_flag && control_bit){
		//TODO: Implement wakeup from stop mode logic
	}

}


static void USART_LoadTDR(USART_Handle_t *pUSARTHandle){

	// Define a new variable for readability
	uint8_t *pTxBuffer = pUSARTHandle->pTxBuffer;

	// Check if the USART wordlength is 7, 8 or 9 bits
	if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_7BITS){
		// Load the data into the Transfer Data Register (mask the last bit as 0)
		pUSARTHandle->pUSARTx->TDR = (*pTxBuffer & 0xFE);

		pUSARTHandle->pTxBuffer++;
	}else if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS){
		// Load the data into the Transfer Data Register
		pUSARTHandle->pUSARTx->TDR = (*pTxBuffer & (uint8_t)0xFF);

		pUSARTHandle->pTxBuffer++;
	}else{
		// If 9 bit wordlength load the DR with a 2 byte masking that is 0 for any except the first 9 bits
		uint16_t *pdata = (uint16_t*) pTxBuffer;
		// Load the data into the Transfer Data Register
		pUSARTHandle->pUSARTx->TDR = (*pdata & (uint16_t)0x01FF);

		pUSARTHandle->pTxBuffer++;

		// If the parity check is not disabled 9 bits of user data will be sent therefore the TxBuffer must
		// be incremented twice. Otherwise only 8 bits will be sent and the buffer incremented once (done above)
		// as the 9th bit will be the parity bit
		if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
			pUSARTHandle->pTxBuffer++;
		}
	}

	pUSARTHandle->nTransmitted++;
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


	pUSARTHandle->pUSARTx->CR1 |= tempreg;
	tempreg = 0;

	/********************** Configure BRR Register **********************/
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);

	/********************** Configure CR2 Register **********************/

	// Configure number of stop bits during a USART frame transmission
	tempreg |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USARTx_CR2_STOP);

	pUSARTHandle->pUSARTx->CR2 |= tempreg;
	tempreg = 0;
	/********************** Configure CR3 Register **********************/

	// Configure USART hardware flow control
	// The hardware control macros are set such that the correct values will be placed in the RTSE and CTSE registers
	tempreg |= (pUSARTHandle->USART_Config.USART_HWFlowControl << USARTx_CR3_RTSE);

	// Configure DMA use for Transmitter and Receiver
	tempreg |= (pUSARTHandle->USART_Config.USART_DMATransmitter << USARTx_CR3_DMAT);
	tempreg |= (pUSARTHandle->USART_Config.USART_DMAReceiver << USARTx_CR3_DMAR);
	pUSARTHandle->pUSARTx->CR3 |= tempreg;


}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len){
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate == USART_NOT_BUSY_IN_TX){

		// Link the passed handle to the USART handles stored by the driver
		// If an invalid one is passed then return not busy. an application callback is sent from within the function
		if(USART_SetHandleLink(pUSARTHandle) == false){
			return USART_NOT_BUSY_IN_TX;
		}

		pUSARTHandle->TransmitLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		// Configure the peripheral into transmit mode
		pUSARTHandle->pUSARTx->CR1 |= (USART_MODE_ONLY_TX << USARTx_CR1_RE);

		// Enable the interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (ENABLE << USARTx_CR1_TCIE);

		// Enable the interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (ENABLE << USARTx_CR1_TXEIE);

	}

	return txstate;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
	uint8_t rxstate = pUSARTHandle->TxBusyState;

	if(rxstate == USART_NOT_BUSY_IN_RX){
		pUSARTHandle->TransmitLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;
		pUSARTHandle->nTransmitted = 0;
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

// Links the drivers Usart handles to the handle given by the user
static bool USART_SetHandleLink(USART_Handle_t *pUSARTHandle){
	if(pUSARTHandle->pUSARTx == USART1){
		pUSART1HandleLink = pUSARTHandle;
	}else if(pUSARTHandle->pUSARTx == USART2){
		pUSART2HandleLink = pUSARTHandle;
	}else if(pUSARTHandle->pUSARTx == USART3){
		pUSART3HandleLink = pUSARTHandle;
	}else if(pUSARTHandle->pUSARTx == UART4){
		pUART4HandleLink = pUSARTHandle;
	}else if(pUSARTHandle->pUSARTx == UART5){
		pUART5HandleLink = pUSARTHandle;
	}else{
		USART_ApplicationEventCallback(pUSARTHandle, USART_HANDLE_NOT_VALID);
		return false;
	}

	return true;
}

// Aplication callback
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv){
}
