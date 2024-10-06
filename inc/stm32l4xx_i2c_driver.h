/*
 * stm32l4xx_i2c_driver.h
 *
 *  Created on: 7 Sep 2024
 *      Author: Pillow sleeper
 */

#ifndef INC_STM32L4XX_I2C_DRIVER_H_
#define INC_STM32L4XX_I2C_DRIVER_H_

#include "stm32l4xx.h"

// Configuration structure for I2Cx peripheral
typedef struct
{
	uint8_t	 I2C_DeviceAddress;
	uint8_t	 I2C_NACKControl;
	uint8_t  I2C_SCLL;
	uint8_t	 I2C_SCLH;
	uint8_t	 I2C_SDADEL;
	uint8_t	 I2C_SCLDEL;
	uint8_t	 I2C_PRESC;
	uint8_t  I2C_ANFOFF;
	uint8_t  I2C_DNF;
	uint8_t  I2C_NOSTRECH;
	uint8_t  I2C_GC;
}I2C_Config_t;

// Handle structure for I2Cx peripheral
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t 	 *pTxBuffer;	// Tx Buffer
	uint8_t	 	 *pRxBuffer;	// Rx buffer
	uint32_t 	 TxLen;			// Length of the Tx data
	uint32_t	 nTransmitted;	// Number of data packets transmitted
	uint32_t	 RxLen;			// Length of the Rx data
	uint32_t	 nReceived;		// Number of data packets received
	uint32_t	 TxRxState;		// Is the MU currently transmitting or receiving
	uint8_t		 DevAddr;		// Stores the address of the slave or this device (as a slave)
	uint32_t	 RxSize;		// Size of receiving data
	uint8_t	  	 Sr;			// Repeated start value
}I2C_Handle_t;

// I2C SCL Speed contrl
#define I2C_SCL_SPEED_SM 100000
#define I2C_SCL_SPEED_FM 400000

// I2C Ack Control
#define I2C_NACK_ENABLE 	1	// An ACK is sent after byte received
#define I2C_NACK_DISABLE 	0 	// An NACK is sent after byte received (default)

// I2C Master start read write
#define I2C_MASTER_WRITE 	0
#define I2C_MASTER_READ 	1

// I2C reload
#define I2C_AUTOEND_TRUE 	1
#define I2C_AUTOEND_FALSE 	0

// Busy states
#define I2C_NOT_BUSY		0
#define I2C_BUSY_IN_TX		1
#define I2C_BUSY_IN_RX		2

// I2C related status flags
#define I2C_TXE_FLAG			(1 << I2Cx_ISR_TXE)
#define I2C_TXIS_FLAG			(1 << I2Cx_ISR_TXIS)
#define I2C_RXNE_FLAG			(1 << I2Cx_ISR_RXNE)
#define I2C_ADDR_FLAG			(1 << I2Cx_ISR_ADDR)
#define I2C_NACKF_FLAG			(1 << I2Cx_ISR_NACKF)
#define I2C_STOPF_FLAG			(1 << I2Cx_ISR_STOPF)
#define I2C_TC_FLAG				(1 << I2Cx_ISR_TC)
#define I2C_TCR_FLAG			(1 << I2Cx_ISR_TCR)
#define I2C_BERR_FLAG			(1 << I2Cx_ISR_BERR)
#define I2C_OVR_FLAG			(1 << I2Cx_ISR_OVR)
#define I2C_PECERR_FLAG			(1 << I2Cx_ISR_PECERR)
#define I2C_TIMEOUT_FLAG		(1 << I2Cx_ISR_TIMEOUT)
#define I2C_ALERT_FLAG			(1 << I2Cx_ISR_ALERT)
#define I2C_BUSY_FLAG			(1 << I2Cx_ISR_BUSY)
#define I2C_DIR_FLAG			(1 << I2Cx_ISR_DIR)

// Application events

#define I2C_EV_RX_FAILED		1
#define I2C_EV_RX_CMPLT			2
#define I2C_EV_TX_FAILED		3
#define I2C_EV_TX_CMPLT			4
#define I2C_EQ_INVALID_HANDLE	5
#define I2C_EQ_BUSY				6
//------------------------------------------------------------FUNCTION DEFINITIONS------------------------------------------------------------//

// Peripheral clock set up
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

// Init and De-init
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

// Data Send and Receive

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t slaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t slaveAddr);

void I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t slaveAddr);
void I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t slaveAddr);

// IRQ Configuration and ISR handling
void I2C_IRQInterruptControl(I2C_Handle_t *pI2CHandle, uint8_t EnOrDi);
void I2C1_EV_IRQHandler();
void I2C2_EV_IRQHandler();
void I2C3_EV_IRQHandler();
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

// Other peripheral Control APIs
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
uint8_t I2C_VerifyResponse(uint8_t ackByte);

// Aplication callback
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);
#endif /* INC_STM32L4XX_I2C_DRIVER_H_ */
