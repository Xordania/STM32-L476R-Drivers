/*
 * stm32l4xx_I2C_driver.c
 *
 *  Created on: 7 Sep 2024
 *      Author: Pillow sleeper
 */

#include "stm32l4xx_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx, uint8_t addr, uint8_t RorW);
static void I2C_FillNBytes(I2C_Handle_t *pI2CHandle, uint8_t len);
static void I2C_EV_IRQHandler(I2C_Handle_t *pI2CHandle);

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB_PreScaler[4] = {2, 4, 8, 16};

uint32_t MSI_RANGES[12] = {MSI_RANGE_0, MSI_RANGE_1, MSI_RANGE_2, MSI_RANGE_3, MSI_RANGE_4, MSI_RANGE_5,MSI_RANGE_6, MSI_RANGE_7, MSI_RANGE_8, MSI_RANGE_9, MSI_RANGE_10, MSI_RANGE_11};

I2C_Handle_t *pI2C1HandleLink;
I2C_Handle_t *pI2C2HandleLink;
I2C_Handle_t *pI2C3HandleLink;

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	}else{
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}else if(pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}
	}
}

// Enable the peripheral
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pI2Cx->CR1 |= (1 << I2Cx_CR1_PE);
	}else{
		pI2Cx->CR1 &= ~(1 << I2Cx_CR1_PE);
	}
}

// Find out the frequency of the clock being used as the system clock
uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1;

	uint8_t clksrc, temp, ahbp, apb;

	clksrc = (RCC->CFGR >> 2) & 0x3;

	if(clksrc == 0x0){
		// Clock source is MSI
		uint8_t MSI_Range;
		uint8_t	MSIRGSEL = (RCC->CR >> 3) & 0x1;

		// Get MSI_Range from the correct register
		if(MSIRGSEL == 1){
			MSI_Range = (RCC->CR >> 4) & 0xF;
		} else if(MSIRGSEL == 0){
			MSI_Range = (RCC->CSR >> 8) & 0xF;
		}
		pclk1 = MSI_RANGES[MSI_Range];
	} else if(clksrc == 0x1){
		// Clock source is 16MHz HSI16
		pclk1 = HSI16_Clock_Frequency;
	} else if(clksrc == 0x2){
		// Clock source is HSE, an external clock source, this section is incomplete
		// TODO: Fill this out

	} else if(clksrc == 0x03){
		// Clock source is PLL, this looks like a pain to fill our, I'm not doing it for now
		// TODO: Fill this out

	}

	// Get the register value that defines the AHB pre-scaler
	temp = (RCC->CFGR >> 4) & 0xF;

	if(temp < 8){
		ahbp = 1;
	}else{
		ahbp = AHB_PreScaler[temp - 8];
	}

	// Get the register value that defines the APB1 pre-scaler
	temp = (RCC->CFGR >> 8) & 0x7;

	if(temp > 4){
		apb = 1;
	}else{
		apb = APB_PreScaler[temp - 4];
	}

	return pclk1/(apb * ahbp);
}


void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t tempreg = 0;

	// Turn on the clock going to the I2C peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// Clear the CR1, CR2, TIMINGR, OAR1 and OAR2 registers
	pI2CHandle->pI2Cx->CR1 &= 0;
	pI2CHandle->pI2Cx->CR2 &= 0;
	pI2CHandle->pI2Cx->TIMINGR &= 0;
	pI2CHandle->pI2Cx->OAR1 &= 0;
	pI2CHandle->pI2Cx->OAR2 &= 0;


	// Configure ANFOFF register
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_ANFOFF << I2Cx_CR1_ANFOFF);
	pI2CHandle->pI2Cx->CR1 |= tempreg;

	// Configure DNF
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DNF << I2Cx_CR1_DNF);
	pI2CHandle->pI2Cx->CR1 |= tempreg;

	// NACK control bit
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_NACKControl << I2Cx_CR2_NACK);
	pI2CHandle->pI2Cx->CR2 |= tempreg;

	// Configure the TIMINGR registers
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_SCLL << I2Cx_TIMINGR_SCLL);
	pI2CHandle->pI2Cx->TIMINGR |= tempreg;

	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_SCLH << I2Cx_TIMINGR_SCLH);
	pI2CHandle->pI2Cx->TIMINGR |= tempreg;

	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_SDADEL << I2Cx_TIMINGR_SDADEL);
	pI2CHandle->pI2Cx->TIMINGR |= tempreg;

	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_SCLDEL << I2Cx_TIMINGR_SCLDEL);
	pI2CHandle->pI2Cx->TIMINGR |= tempreg;

	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_PRESC << I2Cx_TIMINGR_PRESC);
	pI2CHandle->pI2Cx->TIMINGR |= tempreg;

	// Configure register for slave mode

	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_NOSTRECH << I2Cx_CR1_NOSTRECH);
	pI2CHandle->pI2Cx->CR1 |= tempreg;

	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_GC << I2Cx_CR1_GCEN);
	pI2CHandle->pI2Cx->TIMINGR |= tempreg;

	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_SBC << I2Cx_CR1_SBC);
	pI2CHandle->pI2Cx->TIMINGR |= tempreg;

	// Program the address of the device (using 7-bit address)
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << I2Cx_OAR1_OA1);
	pI2CHandle->pI2Cx->OAR1 |= tempreg;

	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_OA2 << I2Cx_OAR2_OA2);
	pI2CHandle->pI2Cx->TIMINGR |= tempreg;


}


void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	if(pI2Cx == I2C1){
		I2C1_PCLK_DI();
	}else if(pI2Cx == I2C2){
		I2C2_PCLK_DI();
	}else if(pI2Cx == I2C3){
		I2C3_PCLK_DI();
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->ISR & FlagName){
		return FLAG_SET;
	}

	return FLAG_RESET;
}


void I2C_IRQInterruptControl(I2C_Handle_t *pI2CHandle, uint8_t EnOrDi){
	// Initialise all of the I2C interrupts

	// Put a 1 in all of the interrupt activate bits
	if(EnOrDi == ENABLE){
		pI2CHandle->pI2Cx->CR1 |= (0x7F << ENABLE);
	} else if(EnOrDi == DISABLE){
		pI2CHandle->pI2Cx->CR1 &= ~(0x7F << ENABLE);
	}
}
/*
 * IRQ Configuration and ISR handling
 */
/*****************************************************************
 * @fn          - I2C_IRQInterruptConfig
 *
 * @brief       - This function configures interrupt
 *
 * @param[in]   - IRQ Interrupt number
 * @param[in]   - Macro: Enable/Disable
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
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

/*****************************************************************
 * @fn          - I2C_IRQPriorityConfig
 *
 * @brief       - This function configures interrupt priority
 *
 * @param[in]   - IRQ Interrupt number
 * @param[in]   - IRQ interrupt priority
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}



void I2C1_EV_IRQHandler(void){
	I2C_EV_IRQHandler(pI2C1HandleLink);
}

void I2C2_EV_IRQHandler(void){
	I2C_EV_IRQHandler(pI2C2HandleLink);
}

void I2C3_EV_IRQHandler(void){
	I2C_EV_IRQHandler(pI2C3HandleLink);
}

static void I2C_EV_IRQHandler(I2C_Handle_t *pI2CHandle){
	//Interrupt handling for both master and slave mode of device

	// To check whether it is a specific flag that creates an interrupt we have to check
	// both the event flagand the control bit
	uint8_t event_flag, control_bit;
	event_flag = pI2CHandle->pI2Cx->CR1 & (1 << I2Cx_CR1_RXIE);
	control_bit = pI2CHandle->pI2Cx->ISR & (1 << I2Cx_ISR_RXNE);

	//1. Handle interrupt generated by RXNE event
	if(event_flag && control_bit){
		//RXNE flag is set


		// Check to see if the Microcontroller is currently acting a master or slave
		if(pI2CHandle->SorM == I2C_MASTER_MODE){

			// Read from the Rx buffer and increment the pointer
			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->RXDR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->nReceived++;

			// If NBYTES have been transmitted and a repeated start is not required let the
			//application know the receiving has finished successfully
			if(pI2CHandle->nReceived == pI2CHandle->RxLen){
				if(pI2CHandle->Sr == 0){
					pI2CHandle->TxRxState = I2C_NOT_BUSY;
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
				}
			}
		}else if(pI2CHandle->SorM == I2C_SLAVE_MODE){
			// Called when data is being sent to the slave
			if(pI2CHandle->TxRxState == I2C_SLAVE_WRITE){
				// Let the application know that data is being received
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
	event_flag = pI2CHandle->pI2Cx->CR1 & (1 << I2Cx_CR1_TXIE);
	control_bit = pI2CHandle->pI2Cx->ISR & (1 << I2Cx_ISR_TXIS);

	//2. Handle interrupt generated by TXIS event
	if(event_flag && control_bit){
		//TXIS flag is set

		// Check to see if the Microcontroller is currently acting a master or slave
		if(pI2CHandle->SorM == I2C_MASTER_MODE){

			// Load the next byte into Tx buffer and increment the pointer
			// This clears the TXIS flag
			pI2CHandle->pI2Cx->TXDR = *pI2CHandle->pTxBuffer;
			pI2CHandle->pTxBuffer++;
			pI2CHandle->nTransmitted++;


			// If NBYTES have been transmitted and a repeated start is not required let the
			//application know the transmit has finished successfully
			if(pI2CHandle->nTransmitted == pI2CHandle->TxLen){
				if(pI2CHandle->Sr == 0){
					pI2CHandle->TxRxState = I2C_NOT_BUSY;
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}else if(pI2CHandle->SorM == I2C_SLAVE_MODE){
			// Called when data is requested of the slave
			if(pI2CHandle->TxRxState == I2C_SLAVE_WRITE){
				// Let the application know that data is being requested
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}

	}
	event_flag = pI2CHandle->pI2Cx->CR1 & (1 << I2Cx_CR1_STOPIE);
	control_bit = pI2CHandle->pI2Cx->ISR & (1 << I2Cx_ISR_STOPF);

	//3. Handle interrupt generated by STOP event
	// This is set if as a master the MC sends a stop or if one is received as a slave
	if(event_flag && control_bit){
		// STOP flag is set

		// Clear the stop flag
		pI2CHandle->pI2Cx->ICR |= (ENABLE << I2Cx_ICR_STOPCF);

		// Set handle to not busy
		pI2CHandle->TxRxState = I2C_NOT_BUSY;

		// Send application event
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	event_flag = pI2CHandle->pI2Cx->CR1 & (1 << I2Cx_CR1_TCIE);
	control_bit = pI2CHandle->pI2Cx->ISR & (1 << I2Cx_ISR_TCR);

	//4. Handle interrupt generated by TCR
	//NOTE: Master mode only
	if(event_flag && control_bit){
		//TCR flag is set

		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			pI2CHandle->TxRxState = I2C_NOT_BUSY;
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
		}else{
			pI2CHandle->TxRxState = I2C_NOT_BUSY;
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
		}
	}

	event_flag = pI2CHandle->pI2Cx->CR1 & (1 << I2Cx_CR1_TCIE);
	control_bit = pI2CHandle->pI2Cx->ISR & (1 << I2Cx_ISR_TC);

	//5. Handle interrupt generated by TC event
	if(event_flag && control_bit){
		// TC flag is set
		// This will only get called if AUTOEND is set to 1. So a stop is automatticaly sent

		// Inform the applicationt that the current transaction is complete
		if(pI2CHandle->nTransmitted == pI2CHandle->TxLen
				&& pI2CHandle->TxRxState == I2C_BUSY_IN_TX){

			pI2CHandle->TxRxState = I2C_NOT_BUSY;

			// Stops this if statement being called in case another interrupt fires
			pI2CHandle->nTransmitted = 0;
			pI2CHandle->TxLen = 0;

			// Let the application know that the transmission has been complete
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
		}else if(pI2CHandle->nReceived == pI2CHandle->RxLen
				&& pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			pI2CHandle->TxRxState = I2C_NOT_BUSY;

			// Stops this if statement being called in case another interrupt fires
			pI2CHandle->nReceived = 0;
			pI2CHandle->RxLen = 0;

			// Let the application know that the reception has been complete
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);

		}

		// Disable the interrupt, this does not set the TC flag to zero
		// Doing this stops the driver getting stuck in an infinite loop
		// where it constatnly calls this function
		pI2CHandle->pI2Cx->CR1 &= ~(ENABLE << I2Cx_CR1_TCIE);

	}

	event_flag = pI2CHandle->pI2Cx->CR1 & (1 << I2Cx_CR1_ADDRIE);
	control_bit = pI2CHandle->pI2Cx->ISR & (1 << I2Cx_ISR_ADDR);

	//6. Handle interrupt generated by ADDRIE event
	// NOTE: This will only wake up the device from Stop mode if the I2C instance
	// suports the Wakeup from Stop mode feature (all for Stop Mode 1. Only I2C3 for Stop Mode 2)
	// slave mode only
	if(event_flag && control_bit){
		//ADDR flag is set
		if(pI2CHandle->I2C_Config.I2C_NOSTRECH){

		}else{
			// Read the ADDCODE
			uint8_t add_code = (pI2CHandle->pI2Cx->ISR & I2Cx_ISR_ADDCODE);
			uint8_t dir = (pI2CHandle->pI2Cx->ISR & I2Cx_ISR_DIR);

			if(add_code == (pI2CHandle->I2C_Config.I2C_OA1)){
				// Address matches OA1
				I2C_SlaveOA1MatchCallback(pI2CHandle);

			}else{
				// Address matches OA2
				I2C_SlaveOA2MatchCallback(pI2CHandle);
			}

			if(dir == I2C_SLAVE_WRITE){
				// Put slave in transmit mode
				// It is assumed that the slave has already been told what to transmit so the buffer is full
				pI2CHandle->TxRxState = I2C_SLAVE_WRITE;
			}else{
				// Put slave into receiver mode
				pI2CHandle->TxRxState = I2C_SLAVE_READ;
			}

			// Clear the ADDRCF, doing this stops the clock being streched
			pI2CHandle->pI2Cx->ICR &= (ENABLE << I2Cx_ICR_ADDRCF);
		}
	}

	event_flag = pI2CHandle->pI2Cx->CR1 & (1 << I2Cx_CR1_NACKIE);
	control_bit = pI2CHandle->pI2Cx->ISR & (1 << I2Cx_ISR_NACKF);

	//7. Handle interrupt generated by NACKIE event
	if(event_flag && control_bit){
		//NACKF flag is set

		// Clear the NACK flag
		pI2CHandle->pI2Cx->ICR |= (ENABLE << I2Cx_ICR_NACKCF);

		// Send a message to the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_NACK);
	}

}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){
	//Interrupt handling for both master and slave mode of device

	//1. Handle interrupt generated by BERR event

	//2. Handle interrupt generated by ARLO event

	//3. Handle interrupt generated by OVR event

	//4. Handle interrupt generated by PECERR event

	//5. Handle interrupt generated by TIMEOUT event

	//6. Handle interrupt generated by ALERT event
}


// TODO: Need to test that multi-start (RELOAD) transmit and receive work
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t slaveAddr){

	do{
		uint8_t sLen;

		pI2CHandle->SorM = I2C_MASTER_MODE;

		// Set the RELOAD bit to nengative, else a start condition cannot be sent
		pI2CHandle->pI2Cx->CR2 &= ~(ENABLE << I2Cx_CR2_RELOAD);

		//2. Check if Len is greater than 255, if it is then we need to set the Autoend bit, fill NBYTES and generate start condition
		if(Len > 255){
			// Set sections length to 255
			sLen = 255;

			//3. Set the NBYTES registers to the number of bytes to be transmitted
			pI2CHandle->pI2Cx->CR2 |= (sLen << I2Cx_CR2_NBYTES);


			//Set the Reload register to true
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx, slaveAddr, I2C_MASTER_WRITE);
			pI2CHandle->pI2Cx->CR2 |= (ENABLE << I2Cx_CR2_RELOAD);


		}else{
			// Set section length equal to length
			sLen = Len;

			//3. Set the NBYTES registers to the number of bytes to be transmitted
			pI2CHandle->pI2Cx->CR2 |= (sLen<< I2Cx_CR2_NBYTES);

			I2C_GenerateStartCondition(pI2CHandle->pI2Cx, slaveAddr, I2C_MASTER_WRITE);

		}


		// Minus by the amount the MC is about to send. Saves minusing every iteration of the loop
		Len = Len - sLen;

		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG));

		//5. When Len becomes zero wait for TC flag in ISR and NBYTES have been sent
		// 	 The SCL is streched low until the TC flag in cleared by sending a NBYTEs of data
		while(sLen > 0){

			pI2CHandle->pI2Cx->TXDR = *pTxBuffer;
			pTxBuffer++;
			sLen--;

			// Wait for the TXE flag to be set otherwise it might send the stop signal too soon and miss the last character
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG));
		}


	}while(Len > 0);

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TC_FLAG));

	//6. Generate a STOP condition, clearing the TC flag
	pI2CHandle->pI2Cx->CR2 |= (1 << I2Cx_CR2_STOP);
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len , uint8_t slaveAddr){
	do{
		uint8_t sLen;

		pI2CHandle->SorM = I2C_MASTER_MODE;

		//2. Check if Len is greater than 255, if it is then we need to set the Autoend bit, fill NBYTES and generate start condition
		if(Len > 255){
			// Set sections length to 255
			sLen = 255;

			//3. Set the NBYTES registers to the number of bytes to be transmitted
			pI2CHandle->pI2Cx->CR2 |= (sLen << I2Cx_CR2_NBYTES);

			// If the RELOAD register is not set this must be the first set of data. Send a start
			if(!(pI2CHandle->pI2Cx->CR2 & (ENABLE << I2Cx_CR2_RELOAD))){
				//Set the Reload register to true
				pI2CHandle->pI2Cx->CR2 |= (ENABLE << I2Cx_CR2_RELOAD);
				I2C_GenerateStartCondition(pI2CHandle->pI2Cx, slaveAddr, I2C_MASTER_READ);
			}

		}else{
			// Set section length equal to length
			sLen = Len;

			//3. Set the NBYTES registers to the number of bytes to be transmitted
			pI2CHandle->pI2Cx->CR2 |= (sLen<< I2Cx_CR2_NBYTES);

			// Check if the reload bit is set. If it is then this is not the first set of data from the TX. Zero the Reload bit
			// Otherwise send a start message
			if(pI2CHandle->pI2Cx->CR2 & (ENABLE << I2Cx_CR2_RELOAD)){
				//Clear the reload register setting it false
				pI2CHandle->pI2Cx->CR2 &= ~(ENABLE << I2Cx_CR2_RELOAD);
			}else{
				I2C_GenerateStartCondition(pI2CHandle->pI2Cx, slaveAddr, I2C_MASTER_READ);
			}
		}

		// Minus by the amount the MC is about to send. Saves minusing every iteration of the loop
		Len = Len - sLen;

		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));


		//5. Keep receiving data from the slave until Len becomes zero, then send a stop
		//	 After each bit has been received an acknowledge is sent provided until NBYTES have
		// 	 been received
		while(sLen > 0){

			// Check the RXNE. When it is set there is stuff to be received in the buffer
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));

			// Copy the contents of the register into the buffer
			*pRxBuffer = pI2CHandle->pI2Cx->RXDR;
			pRxBuffer++;
			sLen--;
		}

		//6. Check the transfer is complete by seeing the TC flag is set
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TC_FLAG));
	}while(Len > 0);

	//7. Generate a STOP condition, clearing the TC flag
	pI2CHandle->pI2Cx->CR2 |= (1 << I2Cx_CR2_STOP);

}

void I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t slaveAddr){
	uint8_t busystate = pI2CHandle->TxRxState;

	pI2CHandle->SorM = I2C_MASTER_MODE;

	//1. Check to see if the passed handle is already busy with an I2C transaction
	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)){

		//2. Set the current I2C transmitting handle to the given handle
		if(pI2CHandle->pI2Cx == I2C1){
			pI2C1HandleLink = pI2CHandle;
		}else if(pI2CHandle->pI2Cx == I2C2){
			pI2C2HandleLink = pI2CHandle;
		}else if(pI2CHandle->pI2Cx == I2C3){
			pI2C3HandleLink = pI2CHandle;
		}else{
			// If the passed handle does not contain a proper pointer to an I2C register address
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EQ_INVALID_HANDLE);
			return;
		}

		//3. Populate the handle with the current transactions parameters
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->nTransmitted = 0;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = slaveAddr;

		//4. Fill the NBytes register, letting the hardware know how many bits are to be transmitted
		I2C_FillNBytes(pI2CHandle, Len);

		//5. Generate the start condition if one has not already been sent before hand
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx, slaveAddr, I2C_MASTER_WRITE);

		//6. Turn the TCIE interrupt on
		pI2C1HandleLink->pI2Cx->CR1 |= (ENABLE << I2Cx_CR1_TCIE);


		//7. Check if this is the final transaction before the master is done with the slave
		if(pI2CHandle->Sr != 0){
			pI2CHandle->pI2Cx->CR2 &= ~(ENABLE << I2Cx_CR2_AUTOEND);
		}else{
			pI2CHandle->pI2Cx->CR2 |= (ENABLE << I2Cx_CR2_AUTOEND);
		}

	}else{
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EQ_BUSY);
	}

}

void I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t slaveAddr){
	uint8_t busystate = pI2CHandle->TxRxState;

	pI2CHandle->SorM = I2C_MASTER_MODE;

	//1. Check to see if the passed handle is already busy with an I2C transaction
	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)){

		//2. Set the current I2C transmitting handle to the given handle
		if(pI2CHandle->pI2Cx == I2C1){
			pI2C1HandleLink = pI2CHandle;
		}else if(pI2CHandle->pI2Cx == I2C2){
			pI2C2HandleLink = pI2CHandle;
		}else if(pI2CHandle->pI2Cx == I2C3){
			pI2C3HandleLink = pI2CHandle;
		}else{
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EQ_INVALID_HANDLE);
		}

		//3. Populate the handle with the current transactions parameters
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->nReceived = 0;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = slaveAddr;

		//4. Fill the NBytes register, letting the hardware know how many bits are to be received
		I2C_FillNBytes(pI2CHandle, Len);

		//5. Generate the start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx, slaveAddr, I2C_MASTER_READ);

		//6. Turn the TCIE interrupt on
		pI2C1HandleLink->pI2Cx->CR1 |= (ENABLE << I2Cx_CR1_TCIE);

		//7. Check if this is the final transaction before the master is done with the slave
		if(pI2CHandle->Sr != 0){
			pI2CHandle->pI2Cx->CR2 &= ~(ENABLE << I2Cx_CR2_AUTOEND);
		}else{
			pI2CHandle->pI2Cx->CR2 |= (ENABLE << I2Cx_CR2_AUTOEND);
		}
	}else{
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EQ_BUSY);
	}
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx, uint8_t addr, uint8_t RorW){
	// Shift the address and clear the zeroth bit as this is only for 7-bit addresses
	addr = addr << 1;
	addr &= ~(1);

	// Set the slave address
	pI2Cx->CR2 &= ~(0x3FF << I2Cx_CR2_SADD);
	pI2Cx->CR2 |= (addr << I2Cx_CR2_SADD);

	// Set the bit to determine whether read or write
	pI2Cx->CR2 &= ~(1 << I2Cx_CR2_RD_WRN);
	pI2Cx->CR2 |= (RorW << I2Cx_CR2_RD_WRN);

	// Set the START generation bit. This is automatically cleared by the hardware
	pI2Cx->CR2 |= (1 << I2Cx_CR2_START);
}


// Fills the RELOAD bit if more than 8 bytes need to be sent
static void I2C_FillNBytes(I2C_Handle_t *pI2CHandle, uint8_t len){
	if(len > 255){
		pI2CHandle->pI2Cx->CR2 |= (255 << I2Cx_CR2_NBYTES);
		pI2CHandle->pI2Cx->CR2 |= (ENABLE << I2Cx_CR2_RELOAD);

	}else{
		pI2CHandle->pI2Cx->CR2 |= (len << I2Cx_CR2_NBYTES);

	}

}


void I2C_SlaveIntialization(I2C_Handle_t *pI2CHandle){
	// Initial settings handled in I2C_Init()

	// Set handle into slave mode
	pI2CHandle->SorM = I2C_SLAVE_MODE;

	// Clear OA1EN and OA2EN
	pI2CHandle->pI2Cx->OAR1 &= ~(ENABLE << I2Cx_OAR1_OA1EN);
	pI2CHandle->pI2Cx->OAR2 &= ~(ENABLE << I2Cx_OAR2_OA2EN);

	// Configure OA1, OA2, O1EN, OA2EN.
	// Only 7-bit addressing and no OA2MSK taken into account

	// Clear OA1 before setting it
	pI2CHandle->pI2Cx->OAR1 &= ~(0x3FF << I2Cx_OAR1_OA1);
	pI2CHandle->pI2Cx->OAR1 |= (pI2CHandle->pI2Cx->OAR1 << I2Cx_OAR1_OA1);

	// Clear OA2 before setting it
	pI2CHandle->pI2Cx->OAR2 &= ~(0x3FF << I2Cx_OAR2_OA2);
	pI2CHandle->pI2Cx->OAR2 |= (pI2CHandle->pI2Cx->OAR2 << I2Cx_OAR2_OA2);

	// Enable OA1EN and OA2En
	pI2CHandle->pI2Cx->OAR1 |= (ENABLE << I2Cx_OAR1_OA1EN);
	pI2CHandle->pI2Cx->OAR2 |= (ENABLE << I2Cx_OAR2_OA2EN);

	// SBC cleared and configured in I2C_Init()

	// Intialization of interrupts is assumed to be taken care of by the application
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data){
	pI2C->TXDR |= data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C){
	return (pI2C->RXNE & 0xf);
}

// A function that gets called whenver there in a match of address OA1 in slave mode
__weak void I2C_SlaveOA1MatchCallback(I2C_Handle_t *pI2CHandle){

}

// A function that gets called whenver there in a match of address OA2 in slave mode
__weak void I2C_SlaveOA2MatchCallback(I2C_Handle_t *pI2CHandle){

}

// Aplication callback
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv){
}

