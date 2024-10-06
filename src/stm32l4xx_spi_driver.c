/*
 * stm32l4xx_spi_driver.c
 *
 *  Created on: 29 Mar 2024
 *      Author: Pillow sleeper
 */

#include "stm32l4xx_spi_driver.h"

// Private helper functions
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

#include <math.h>
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
	} else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}
	}
}


void SPI_Init(SPI_Handle_t *pSPIHandle){
	// Configure SPI_CR1 Register

	uint16_t tempreg = 0;

	// 0. Enable peripheral clock

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// 1. Configure device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPIx_CR1_MSTR;

	// 2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// bidi mode should be cleared
		tempreg &= ~(1 << SPIx_CR1_BIDI_MODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// bidi mode should be set
		tempreg |= (1 << SPIx_CR1_BIDI_MODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		// bidi mode should be cleared
		tempreg &= ~(1 << SPIx_CR1_BIDI_MODE);

		// RXOnly mode should be set
		tempreg |= (1 << SPIx_CR1_RX_ONLY);

	}

	// 2. Configure SSM mode
	if(pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_EN){
		// Set SSM
		tempreg |= (1 << SPIx_CR1_SSM);
	}else if(pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_DI){
		// Clear SSM
		tempreg &= ~(1 << SPIx_CR1_SSM);
	}

	// 3. Configure CPHA
	if(pSPIHandle->SPIConfig.SPI_CPHA == SPI_CPHA_HIGH){
		// Set CPHA
		tempreg |= (1 << SPIx_CR1_CPHA);
	}else if(pSPIHandle->SPIConfig.SPI_CPHA == SPI_CPHA_LOW){
		// Clear CPHA
		tempreg &= ~(1 << SPIx_CR1_CPHA);
	}

	// 4. Configure CPOL
	if(pSPIHandle->SPIConfig.SPI_CPOL == SPI_CPOL_HIGH){
		// Set CPOL
		tempreg |= (1 << SPIx_CR1_CPOL);
	}else if(pSPIHandle->SPIConfig.SPI_CPOL == SPI_CPOL_LOW){
		// Clear CPOL
		tempreg &= ~(1 << SPIx_CR1_CPOL);
	}

	// 5. Configure CRCL
	if(pSPIHandle->SPIConfig.SPI_CRCL == SPI_CRCL_16BITS){
		// Set CRCL
		tempreg |= (1 << SPIx_CR1_CRCL);
	}else if(pSPIHandle->SPIConfig.SPI_CRCL == SPI_CRCL_8BITS){
		// Clear CRCL
		tempreg &= ~(1 << SPIx_CR1_CRCL);
	}

	// Set BaudRate bits
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPIx_CR1_BR);

	// Also sets the CR2 register as that follows on directly
	pSPIHandle->pSPIx->CR1 = tempreg;


	// Configurations for CR2

	uint16_t tempreg2 = 0;

	// 6. Configure DS (DS is in CR2 so adding 16-bits to the shift)
	// -1 because the binary values corresponding to the bits wanted are 1 smaller
	tempreg2 |= ((pSPIHandle->SPIConfig.SPI_DS) << SPIx_CR2_DS);
	pSPIHandle->pSPIx->CR2 = tempreg2;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1){
		SPI1_PCLK_DI();
	} else if(pSPIx == SPI2){
		SPI2_PCLK_DI();
	} else if(pSPIx == SPI3){
		SPI3_PCLK_DI();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName){
	if(pSPIx->SR &flagName){
		return FLAG_SET;
	}else{
		return FLAG_RESET;
	}
}

// Data second. This is a blocking call
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len){
	while(len > 0){
		// 1. Wait until the TXbuffer is exmpty
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// Gives in numbers the amount of bits being sent in every message
		uint8_t selectedDS = ((pSPIx->CR2 >> SPIx_CR2_DS) & 15) + 1;

		// 2. Check DS bit in CR2
		if(selectedDS > 8){
			// 16 bit DS
			// Load data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			if(len == 1){
				len = 0;
			}else{
				len = len - 2;
			}
			(uint16_t*)pTxBuffer++;
		} else {
			// 8 bit DS
			// Load data into the DR
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}
	}
}


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len){
	while(len > 0){
		// 1. Wait until the RXbuffer is expty
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		// 2. Check DS bit in CR2
		if((pSPIx->CR2 & (1 << SPIx_CR2_DS))){
			// 16 bit DS
			// Load data from the DR
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			len = len - 2;
			(uint16_t*)pRxBuffer++;
		} else {
			// 8 bit DS
			// Load data from the DR
			*pRxBuffer = pSPIx->DR;
			len--;
			pRxBuffer++;
		}
	}
}
/*
 * As send data via interrupt does not require a for or while loop it is none blocking.
 * It simply saves the data to a TXBuffer that will be emptied when the ISR code is ready.
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){
	uint8_t state = pSPIHandle -> TxState;

	if(state != SPI_BUSY_IN_TX){
		// 1. Save the Tx buffer address and Len infomration in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		// 2. Mark the SPI state as busy in transmission so that no other code can
		// take over the same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit to get interrupt whenever the TXE flag is set in the SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPIx_CR2_TXEIE);

		// 4. Data Transmission will be handled by the ISR code
	}

	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){
	uint8_t state = pSPIHandle -> RxState;

	if(state != SPI_BUSY_IN_RX){
		// 1. Save the Rx buffer address and Len infomration in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		// 2. Mark the SPI state as busy in transmission so that no other code can
		// take over the same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. Enable the TXEIE control bit to get interrupt whenever the RXE flag is set in the SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPIx_CR2_RXNEIE);

		// 4. Data Transmission will be handled by the ISR code
	}

	return state;
}



void SPI_IRQHandling(SPI_Handle_t *pHandle){
	uint8_t temp1, temp2;

	// Check TXE registers
	temp1 = pHandle->pSPIx->SR & (1 << SPIx_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPIx_CR2_TXEIE);

	if(temp1 && temp2){
		// Handle TXE
		spi_txe_interrupt_handle(pHandle);

	}

	// Check RXNE registers
	temp1 = pHandle->pSPIx->SR & (1 << SPIx_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPIx_CR2_RXNEIE);

	if(temp1 && temp2){
		// Handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// Check for ovr flag
	temp1 = pHandle->pSPIx->SR & (1 << SPIx_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPIx_CR2_ERRIE);

	if(temp1 && temp2){
		// Handle RXNE
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SPIx_CR1_SPE);
	}else{
		pSPIx->CR1 &= ~(1 << SPIx_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SPIx_CR1_SSI);
	}else{
		pSPIx->CR1 &= ~(1 << SPIx_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR2 |= (1 << SPIx_CR2_SSOE);
	}else{
		pSPIx->CR2 &= ~(1 << SPIx_CR2_SSOE);
	}
}

uint8_t SPI_VerifyResponse(uint8_t ackByte){
	if(ackByte == 0xf5){
		return 1;
	}else{
		return 0;
	}
}

// Helper funciton implementations
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){

	// Gives in numbers the amount of bits being sent in every message
	uint8_t selectedDS = ((pSPIHandle->pSPIx->CR2 >> SPIx_CR2_DS) & 15) + 1;

	// 2. Check DS bit in CR2
	if(selectedDS > 8){
		// 16 bit DS
		// Load data into the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		if(pSPIHandle->TxLen == 1){
			pSPIHandle->TxLen= 0;
		}else{
			pSPIHandle->TxLen= pSPIHandle->TxLen - 2;
		}
		(uint16_t*)pSPIHandle->pTxBuffer++;
	} else {

		// 8 bit DS
		// Load data into the DR
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer++;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(pSPIHandle->TxLen == 0){
		// TxLen is zero, so close the SPI transmission and inform the application that TX is over
		// This prevents interrupts from setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	// 2. Check DS bit in CR2
	if((pSPIHandle->pSPIx->CR2 & (1 << SPIx_CR2_DS))){
		// 16 bit DS
		// Load data from the DR
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen = pSPIHandle->RxLen - 2;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	} else {
		// 8 bit DS
		// Load data from the DR
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(pSPIHandle->RxLen == 0){
		// RxLen is zero, so close the SPI transmission and inform the apllication that RX is over
		// This prevenets interrupts from setting up the RXNE flag
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){

	uint8_t temp;

	// 1. Clear the ovr flag
	// This is done by reading from the DR and SR registers
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}

	// Stops unused variable warning
	(void)temp;

	// 2. Inform the apllication
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPIx_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPIx_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;

	// Stops unused variable warning
	(void) temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){
	// This is a weak implementation and the application may override this function
}


