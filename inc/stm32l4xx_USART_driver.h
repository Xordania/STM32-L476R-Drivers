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
	uint8_t USART_DMATransmitter;
	uint8_t USART_DMAReceiver;
}USART_Config_t;

// Handle structure for USARTx peripheral
typedef struct{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t TxBusyState;
	uint8_t RxBusyState;
	uint32_t TransmitLen;
	uint32_t ReceiveLen;
	uint32_t nTransmitted;
	uint32_t nReceived;
}USART_Handle_t;

// USART related status flags
#define USARTx_PE_FLAG 		(1 << USARTx_ISR_PE)
#define USARTx_FE_FLAG 		(1 << USARTx_ISR_FE)
#define USARTx_NF_FLAG 		(1 << USARTx_ISR_NF)
#define USARTx_ORE_FLAG 	(1 << USARTx_ISRORE)
#define USARTx_IDLE_FLAG 	(1 << USARTx_ISR_IDLE)
#define USARTx_RXNE_FLAG 	(1 << USARTx_ISR_RXNE)
#define USARTx_TC_FLAG 		(1 << USARTx_ISR_TC)
#define USARTx_TXE_FLAG 	(1 << USARTx_ISR_TXE)
#define USARTx_LBDF_FLAG 	(1 << USARTx_ISR_LBDF)
#define USARTx_CTSIF_FLAG 	(1 << USARTx_ISR_CTSIF)
#define USARTx_CTS_FLAG 	(1 << USARTx_ISR_CTS)
#define USARTx_RTOF_FLAG 	(1 << USARTx_ISR_RTOF)
#define USARTx_EOBF_FLAG 	(1 << USARTx_ISR_EOBF)
#define USARTx_ABRE_FLAG 	(1 << USARTx_ISR_ABRE)
#define USARTx_ABRF_FLAG 	(1 << USARTx_ISR_ABRF)
#define USARTx_BUSY_FLAG 	(1 << USARTx_ISR_BUSY)
#define USARTx_CMF_FLAG 	(1 << USARTx_ISR_CMF)
#define USARTx_SBKF_FLAG 	(1 << USARTx_ISR_SBKF)
#define USARTx_RWU_FLAG 	(1 << USARTx_ISR_RWU)
#define USARTx_WUF_FLAG 	(1 << USARTx_ISR_WUF)
#define USARTx_TEACK_FLAG 	(1 << USARTx_ISR_TEACK)
#define USARTx_REACK_FLAG 	(1 << USARTx_ISR_REACK)


// USART Modes
#define USART_MODE_ONLY_RX 	1
#define USART_MODE_ONLY_TX	2
#define USART_MODE_TXRX		3

// USART Baud rates
#define USART_STD_BAUD_1200		1200
#define USART_STD_BAUD_2400		2400
#define USART_STD_BAUD_9600		9600
#define USART_STD_BAUD_19200	19200
#define USART_STD_BAUD_38400	38400
#define USART_STD_BAUD_57600	57600
#define USART_STD_BAUD_115200	115200
#define USART_STD_BAUD_230400	230400
#define USART_STD_BAUD_460800	460800
#define USART_STD_BAUD_921600	921600
#define USART_STD_BAUD_2MHZ		2000000
#define USART_STD_BAUD_3MHZ		3000000
#define USART_STD_BAUD_4MHZ		4000000

// USART Parity Control
#define USART_PARITY_EN_EVEN	0
#define USART_PARITY_EN_ODD		1
#define USART_PARITY_DISABLE	2

// USART word length
#define USART_WORDLEN_7BITS		0
#define USART_WORDLEN_8BITS		1
#define USART_WORDLEN_9BITS		2

// USART Number of stop bits
#define USART_STOPBITS_1		0
#define USART_STOPBITS_0_5		1
#define USART_STOPBITS_2		2
#define USART_STOPBITS_1_5		3

// USART HW Flow Contrl
#define USART_HW_FLOW_CTRL_NONE		0
#define USART_HW_FLOW_CTRL_RTS		1
#define USART_HW_FLOW_CTRL_CTS		2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

// USART Transmit States
#define USART_NOT_BUSY_IN_TX	0
#define USART_BUSY_IN_TX 		1

// USART Receive States
#define USART_NOT_BUSY_IN_RX	0
#define USART_BUSY_IN_RX		1

// DMA Enable states
#define USART_DMA_DISABLE	0
#define USART_DMA_ENABLE  	1

// Peripheral clock set up
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

// USART control
void USART_USARTControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

// USART Init
void USART_Init(USART_Handle_t *pUSARTHandle);

// Flag statuses
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t FlagName);

// IRQ configration and ISR handling
void USART_IRQInterruptControl(USART_Handle_t *pUSARTHandle, uint8_t EnOrDi);
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

// USART interrupt handlers functions
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void UART4_IRQHandler(void);
void UART5_IRQHandler(void);



// Send and receive data
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

// Configure Baud rate
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

// Application callback
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv);

#endif /* STM32_L476R_DRIVERS_INC_STM32L4XX_USART_DRIVER_H_ */
