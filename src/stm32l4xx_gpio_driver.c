/*
 * stm32l4xx_gpio_driver.c
 *
 *  Created on: 20 Mar 2024
 *      Author: Pillow sleeper
 */

#include "stm32l4xx.h"
#include "stm32l4xx_gpio_driver.h"

// Peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		} else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		} else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		} else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		} else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		} else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		} else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		} else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	}else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		} else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		} else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		} else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		} else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		} else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		} else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		} else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
	}
}

// Init and deinit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp = 0;

	// 0. Enable GPIO clock is enabled

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. Configure the mode of the GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){

		// Non-interrupt mode
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << 2 *  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;


	}else{

			// Set the pins to input mode. This is the default on port A but not on any other ports
			temp = 0 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->MODER &= ~(0x3 << 2 *  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->MODER |= temp;

			// The GPIO pins occupy the EXTI interrupt lines 0-15 corresponding to the pin number that is calling the interrupt
			// Interrupt mode
			if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
				//1. Configure the FTSR
				EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

				// Clear RTSR bit
				EXTI->RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
				//1. Configure the RTSR
				EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

				// Clear FTSR bit
				EXTI->FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


			}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
				//1. Configure both FTSR and RTSR
				EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			}
			else{
				// Set the pins to input mode. This is the default on port A but not on any other ports
				temp = 0 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				pGPIOHandle->pGPIOx->MODER &= ~(0x3 << 2 *  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				pGPIOHandle->pGPIOx->MODER |= temp;
			}
	}

	// 2. configure the GPIO port selection in SYSCFG_EXTICR
	// SYSCFG decides which port takes ove rhte EXTI line

	uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4; // Gets which EXTICR register to use
	uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4; // Calculates which 4 bit section within the register to use
	uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

	// 3. Enable the EXTI interrupt delivery using IMR
	EXTI->IMR1 |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	SYSCFG_PCLK_EN();
	SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

	temp = 0;

	// COFIGURE other pin settings

	// 2. Configure the speed
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << 2 *  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// 3. Configure the pupd settings
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << 2 *  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	// 4. Configure the optype
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType == GPIO_MODE_OUT){
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER |= temp;
	}

	temp = 0;

	// 5. Configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; // Decides which alt function register to access
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << 2 *  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2);

	}
}


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	} else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	} else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	} else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	} else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	} else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	} else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	} else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}
}

// Read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> pinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint8_t value;
	value = (uint8_t)pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value){
	if(value == GPIO_PIN_SET){
		// Write 1 in corresponding bitfield
		pGPIOx->ODR |= (1 << pinNumber);
	}else{
		// Write 0 to corresponding bitfield
		pGPIOx->ODR &= ~(1 << pinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){
	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){
	pGPIOx->ODR ^=(1<< pinNumber);
}

// IRQ configuration and ISR handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64){
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber > 64 && IRQNumber < 96){
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}else{
		if(IRQNumber <= 31){
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64){
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber > 64 && IRQNumber < 96){
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/*
 * Each Interrupt Priority Register(IPR) is 32-bits wide, containing 4 8-bit which
 * defines the priority of that pin. e.g. IPR4 contains the bit fields that hold the
 * pirorty for IRQ16, IRQ17, IRQ18, and IRQ19.
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//1. First find out the ipr register
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber % 4;

	// The Lower four bits of every priority register are not actually implemented so we need to shift to the upper 4 bits
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);


	// Adding 1 to NVIC_PR_BASEADDR moves the address by 32-bits
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}


void GPIO_IRQHandling(uint8_t pinNumber){
	// Clear the EXTI pr register corresponding to the pin number

	if(EXTI->PR1 & (1 << pinNumber)){
		EXTI->PR1 |= (1 << pinNumber);
	}

}
