/*
 * stm32l4xx_spi_driver.h
 *
 *  Created on: 29 Mar 2024
 *      Author: Pillow sleeper
 */

#ifndef INC_STM32L4XX_SPI_DRIVER_H_
#define INC_STM32L4XX_SPI_DRIVER_H_

#include "stm32l4xx.h"

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SSM;
	uint8_t SPI_SSI;
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_CRCL;
	uint8_t SPI_DS;
	uint8_t SPI_SclkSpeed;
}SPI_Config_t;


typedef struct{
	SPI_RegDef_t 	*pSPIx; 		// Holds the base address of the SPIx peripherals
	SPI_Config_t 	SPIConfig;
	uint8_t 		*pTxBuffer; 	// To store the applications TX buffer address
	uint8_t 		*pRxBuffer; 	// To store the applications Rx Buffer address
	uint32_t 		TxLen; 			// To store the Tx len
	uint32_t 		RxLen; 			// To store the Rx Len
	uint8_t 		TxState; 		// To store the Tx state
	uint8_t 		RxState; 		// To store the Rx state
}SPI_Handle_t;

// SPI applcation states
#define SPI_READY							0
#define SPI_BUSY_IN_RX						1
#define SPI_BUSY_IN_TX						2

// Possible SPI apllication events
#define SPI_EVENT_TX_CMPLT					1
#define SPI_EVENT_RX_CMPLT					2
#define SPI_EVENT_OVR_ERR					3
#define SPI_EVENT_CRC_ERR					4

//------------------------------------------------------------MACROS------------------------------------------------------------//

//SPI_DeviceMode
#define SPI_DEVICE_MODE_MASTER 				1
#define SPI_DEVICE_MODE_SLAVE				0

//SPI_BusConfig
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_TXONLY		3
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		4

//SPI_SclkSpeed
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

// SPI_CRCL
#define SPI_CRCL_8BITS 						0
#define SPI_CRCL_16BITS 					1

// SPI_DS
#define SPI_DS_4BITS						3
#define SPI_DS_5BITS						4
#define SPI_DS_6BITS						5
#define SPI_DS_7BITS						6
#define SPI_DS_8BITS						7
#define SPI_DS_9BITS						8
#define SPI_DS_10BITS						9
#define SPI_DS_11BITS						10
#define SPI_DS_12BITS						11
#define SPI_DS_13BITS						12
#define SPI_DS_14BITS						13
#define SPI_DS_15BITS						14
#define SPI_DS_16BITS						15

// SPI_CPOL
#define SPI_CPOL_HIGH						1
#define SPI_CPOL_LOW						0

// SPI_CPHA
#define SPI_CPHA_HIGH						1
#define SPI_CPHA_LOW						0

// SPI_SSM
#define SPI_SSM_EN							1
#define SPI_SSM_DI							0

#define SPI_SSI_EN							1
#define SPI_SSI_DI							0

// SPI related flag definitions
#define SPI_TXE_FLAG 						(1 << SPIx_SR_TXE)
#define SPI_RXNE_FLAG 						(1 << SPIx_SR_RXNE)
#define SPI_CRRCERR_FLAG					(1 << SPIx_SR_CRRCERR)
#define SPI_MODF_FLAG						(1 << SPIx_MODF)
#define SPI_OVR_FLAG						(1 << SPIx_SR_OVR)
#define SPI_BUSY_FLAG						(1 << SPIx_SR_BSY)
#define SPI_FRELVL_FLAG						(1 << SPIx_SR_FRLVL)
#define SPI_FTLVL_FLAG						(1 << SPIx_SR_FTLVL)

//------------------------------------------------------------FUNCTION DEFINITIONS------------------------------------------------------------//


// Peripheral clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

// Init and De-init
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

// Data Send and Receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

// Data Send and Receive with interrupt capabilities
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

// IRQ Configuration and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

// Other peripheral Control APIs
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName);
uint8_t SPI_VerifyResponse(uint8_t ackByte);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

// Aplication callback
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32L4XX_SPI_DRIVER_H_ */
