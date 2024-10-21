/*
 * stm32_l4xx_rcc_driver.c
 *
 *  Created on: 21 Oct 2024
 *      Author: Pillow sleeper
 */

#include "stm32l4xx_rcc_driver.h"


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

