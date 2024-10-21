/*
 * stml4xx.h
 *
 *  Created on: 17 Mar 2024
 *      Author: Pillow sleeper
 */



#ifndef INC_STM32L4XX_H_
#define INC_STM32L4XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))
/*********************************************************** START:Processor Specific Detail ***********************************************************/
/*
 * ARM Cortex M4 Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0 ((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1 ((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2 ((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3 ((__vo uint32_t *)0xE000E10C)
#define NVIC_ISER4 ((__vo uint32_t *)0xE000E110)
#define NVIC_ISER5 ((__vo uint32_t *)0xE000E114)
#define NVIC_ISER6 ((__vo uint32_t *)0xE000E118)
#define NVIC_ISER7 ((__vo uint32_t *)0xE000E11C)

/*
 * ARM Cortex M4 Processor NVIC ICERx register Addresses
 */

#define NVIC_ICER0 ((__vo uint32_t *)0XE000E180)
#define NVIC_ICER1 ((__vo uint32_t *)0xE000E184)
#define NVIC_ICER2 ((__vo uint32_t *)0xE000E188)
#define NVIC_ICER3 ((__vo uint32_t *)0xE000E18C)
#define NVIC_ICER4 ((__vo uint32_t *)0xE000E190)
#define NVIC_ICER5 ((__vo uint32_t *)0xE000E194)
#define NVIC_ICER6 ((__vo uint32_t *)0xE000E198)
#define NVIC_ICER7 ((__vo uint32_t *)0xE000E19C)


/*
 * ARM Cortex M4 Processor Priorty Register Address Calculation
 */

#define NVIC_PR_BASEADDR ((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex M4 Processor the number of priorty bits not implemented
 */

#define NO_PR_BITS_IMPLEMENTED 4


/*********************************************************** END:Processor Specific Detail ***********************************************************/

// Flash memory
#define FLASH_BASEADDR 		0x08000000U
#define SRAM1_BASEADDR 		0x20000000U
#define SRAM2_BASEADDR 		0x20040000U
#define ROM 				0x1FFF8000U
#define SRAM 				SRAM1_BASEADDR

// AHBx and APBx Bus Peripheral base addresses
#define PERIPH_BASE			0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x48000000U


// Base addresses of peripherals hanging on APB1 bus
#define TIM2_BASEADDR 		(APB1PERIPH_BASE + 0x0000U)
#define TIM3_BASEADDR 		(APB1PERIPH_BASE + 0x0400U)
#define TIM4_BASEADDR 		(APB1PERIPH_BASE + 0x0800U)
#define TIM5_BASEADDR 		(APB1PERIPH_BASE + 0x0C00U)
#define TIM6_BASEADDR 		(APB1PERIPH_BASE + 0x1000U)
#define TIM7_BASEADDR 		(APB1PERIPH_BASE + 0x1400U)
#define LCD_BASEADDR 		(APB1PERIPH_BASE + 0x2400U)
#define RTC_BASEADDR 		(APB1PERIPH_BASE + 0x2800U)
#define WWDG_BASEADDR 		(APB1PERIPH_BASE + 0x2C00U)
#define IWDG_BASEADDR 		(APB1PERIPH_BASE + 0x3000U)
#define SPI2_BASEADDR 		(APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASEADDR 		(APB1PERIPH_BASE + 0x3C00U)
#define USART2_BASEADDR 	(APB1PERIPH_BASE + 0x4400U)
#define USART3_BASEADDR 	(APB1PERIPH_BASE + 0x4800U)
#define UART4_BASEADDR 		(APB1PERIPH_BASE + 0x4C00U)
#define UART5_BASEADDR 		(APB1PERIPH_BASE + 0x4C00U)
#define I2C1_BASEADDR 		(APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASEADDR 		(APB1PERIPH_BASE + 0x5800U)
#define I2C3_BASEADDR 		(APB1PERIPH_BASE + 0x5C00U)
#define CAN1_BASEADDR 		(APB1PERIPH_BASE + 0x6400U)
#define PWR_BASEADDR 		(APB1PERIPH_BASE + 0x7000U)
#define DAC1_BASEADDR 		(APB1PERIPH_BASE + 0x7400U)
#define OPAMP_BASEADDR 		(APB1PERIPH_BASE + 0x7800U)
#define LPTIM1_BASEADDR		(APB1PERIPH_BASE + 0x7C00U)
#define LPUART1_BASEADDR	(APB1PERIPH_BASE + 0x8000U)
#define SWPMI1_BASEADDR		(APB1PERIPH_BASE + 0x8800U)
#define LPTIM2_BASEADDR		(APB1PERIPH_BASE + 0x9400U)

// Base addresses of peripherals hanging on APB2 bus
#define SYSCFG_BASEADDR 	(APB2PERIPH_BASE + 0x0000U)
#define VREFBUF_BASEADDR 	(APB2PERIPH_BASE + 0x0030U)
#define COMP_BASEADDR 		(APB2PERIPH_BASE + 0x0200U)
#define EXTI_BASEADDR 		(APB2PERIPH_BASE + 0x0400U)
#define FIREWALL_BASEADDR 	(APB2PERIPH_BASE + 0x1C00U)
#define SDMMC1_BASEADDR 	(APB2PERIPH_BASE + 0x2800U)
#define TIM1_BASEADDR		(APB2PERIPH_BASE + 0x2C00U)
#define SPI1_BASEADDR		(APB2PERIPH_BASE + 0x3000U)
#define TIM8_BASEADDR		(APB2PERIPH_BASE + 0x3400U)
#define USART1_BASEADDR		(APB2PERIPH_BASE + 0x3800U)
#define TIM15_BASEADDR		(APB2PERIPH_BASE + 0x4000U)
#define TIM16_BASEADDR		(APB2PERIPH_BASE + 0x4400U)
#define TIM17_BASEADDR		(APB2PERIPH_BASE + 0x4800U)
#define SAI1_BASEADDR		(APB2PERIPH_BASE + 0x5400U)
#define SAI2_BASEADDR		(APB2PERIPH_BASE + 0x5800U)
#define DFSDM1_BASEADDR		(APB2PERIPH_BASE + 0x6000U)


// Base addresses of peripherals hanging on AHB1 bus
#define DMA1_BASEADDR 		(AHB1PERIPH_BASE + 0x0000U)
#define DMA2_BASEADDR 		(AHB1PERIPH_BASE + 0x0400U)
#define RCC_BASEADDR 		(AHB1PERIPH_BASE + 0x1000U)
#define CRC_BASEADDR 		(AHB1PERIPH_BASE + 0x3000U)
#define TSC_BASEADDR 		(AHB1PERIPH_BASE + 0x4000U)

// Base addresses of peripherals hanging on AHB2 bus
#define GPIOA_BASEADDR 		(AHB2PERIPH_BASE + 0x0000U)
#define GPIOB_BASEADDR 		(AHB2PERIPH_BASE + 0x0400U)
#define GPIOC_BASEADDR 		(AHB2PERIPH_BASE + 0x0800U)
#define GPIOD_BASEADDR 		(AHB2PERIPH_BASE + 0x0C00U)
#define GPIOE_BASEADDR 		(AHB2PERIPH_BASE + 0x1000U)
#define GPIOF_BASEADDR 		(AHB2PERIPH_BASE + 0x1400U)
#define GPIOG_BASEADDR 		(AHB2PERIPH_BASE + 0x1800U)
#define GPIOH_BASEADDR 		(AHB2PERIPH_BASE + 0x1C00U)
#define OTG_FS_BASEADDR 	(AHB2PERIPH_BASE + 0x10000000U)
#define ADC_BASEADDR 		(AHB2PERIPH_BASE + 0x10040000U)
#define AES_BASEADDR 		(AHB2PERIPH_BASE + 0x10060000U)
#define RNG_BASEADDR 		(AHB2PERIPH_BASE + 0x10060800U)

//---------------------------------Peripheral register defintion structures---------------------------------------------
// Specific to the MCU

typedef struct
{
	__vo uint32_t MODER; 			// Sets the configuration of the GPIO port.
	__vo uint32_t OTYPER;			// Sets the output type of the GPIO port (0 = push-pull, 1 = open-drain). Bits 31:16 are reserved.
	__vo uint32_t OSPEEDR;			// The ouput speed of the GPIO port.
	__vo uint32_t PUPDR;			// Dictates what state the pull-up/pull-down resistors are in.
	__vo uint32_t IDR;				// Input Data Registers containing the values coming in for the port. Read-only. Bits 31:16 are reserved.
	__vo uint32_t ODR;				// Output Data Registers. Bits 31:16 are reserved.
	__vo uint32_t BSRR;				// If 15:0 (BS) is set it sets corresponding ODx bit. If 31:16 (BR) are set it resets corresponding ODx bit. BS has priority. All bits write-only.
	__vo uint32_t LCKR;				// Used to lok port bits. When bit 16 is sets the correpsonding bits in 15:0 lock the GPIO configuration. Bits 31:17 are reserved.
	__vo uint32_t AFR[2];			// Sets the alternative functions of the GPIO ports.
	__vo uint32_t BRR;				// Resets the correspondin ODx bit. Bits 31:16 are reserved.
	__vo uint32_t ASCR;				// Configures the analog connection to the IOs. Bits 31:16 are reserved.
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;				// Clock control register
	__vo uint32_t ICSCR;			// Internal clock sources calibration register (
	__vo uint32_t CFGR;				// Clock configuration register
	__vo uint32_t PLLCFGR;          // PLL configuration register (
	__vo uint32_t PLLSAI1CFGR;      // PLLSAI1 configuration register (
	__vo uint32_t PLLSAI2CFGR;      // PLLSAI2 configuration register
	__vo uint32_t CIER;             // Clock interrupt enable register
	__vo uint32_t CIFR;             // Clock interrupt flag register
	__vo uint32_t CICR;             // Clock interrupt clear register
	uint32_t RESERVED0; 			// Reserved memory not to be touched
	__vo uint32_t AHB1RSTR;         // AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;         // AHB2 peripheral reset register
	__vo uint32_t AHB3RSTR;         // AHB3 peripheral reset register
	uint32_t RESERVED1; 			// Reserved memory not to be touched
	__vo uint32_t APB1RSTR1;        // APB1 peripheral reset register 1
	__vo uint32_t APB1RSTR2;        // APB1 peripheral reset register 2
	__vo uint32_t APB2RSTR;         // APB2 peripheral reset register
	uint32_t RESERVED2; 			// Reserved memory not to be touched
	__vo uint32_t AHB1ENR;          // AHB1 peripheral clock enable register
	__vo uint32_t AHB2ENR;          // AHB2 peripheral clock enable register
	__vo uint32_t AHB3ENR;          // AHB3 peripheral clock enable register
	uint32_t RESERVED3; 			// Reserved memory not to be touched
	__vo uint32_t APB1ENR1;         // APB1 peripheral clock enable register 1
	__vo uint32_t APB1ENR2;         // APB1 peripheral clock enable register 2
	__vo uint32_t APB2ENR;          // APB2 peripheral clock enable register
	uint32_t RESERVED4; 			// Reserved memory not to be touched
	__vo uint32_t AHB1SMENR;        // AHB1 peripheral clocks enable in Sleep and Stop modes register
	__vo uint32_t AHB2SMENR;        // AHB2 peripheral clocks enable in Sleep and Stop modes register
	__vo uint32_t AHB3SMENR;        // AHB3 peripheral clocks enable in Sleep and Stop modes register
	uint32_t RESERVED5; 			// Reserved memory not to be touched
	__vo uint32_t APB1SMENR1;       // APB1 peripheral clocks enable in Sleep and Stop modes register 1
	__vo uint32_t APB1SMENR2;       // APB1 peripheral clocks enable in Sleep and Stop modes register 2
	__vo uint32_t APB2SMENR;        // APB2 peripheral clocks enable in Sleep and Stop modes register
	uint32_t RESERVED6; 			// Reserved memory not to be touched
	__vo uint32_t CCIPR;            // Peripherals independent clock configuration register
	uint32_t RESERVED7; 			// Reserved memory not to be touched
	__vo uint32_t BDCR;             // Backup domain control register
	__vo uint32_t CSR;              // Control status register
}RCC_RegDef_t;

typedef struct
{
	__vo uint32_t IMR1;				// Interrupt mask register 1
	__vo uint32_t EMR1;				// Event mask register 1
	__vo uint32_t RTSR1;			// Rising trigger selection register 1
	__vo uint32_t FTSR1;			// Falling trigger selection register 1 (
	__vo uint32_t SWIER1;			// Software interrupt event register 1
	__vo uint32_t PR1;				// Pending register 1
	__vo uint32_t IMR2;				// Interrupt mask register 2
	__vo uint32_t EMR2;				// Event mask register 2
	__vo uint32_t RTSR2;			// Rising trigger selection register 2
	__vo uint32_t FTSR2;			// Falling trigger selection register 2
	__vo uint32_t SWIER2;			// Software interrupt event register 2
	__vo uint32_t PR2;				// Pending register 2
}EXTI_RegDef_t;

typedef struct{
	__vo uint32_t MEMRMP;			// SYSCFG memory remap register
	__vo uint32_t CFGR1;			// SYSCFG configuration register 1
	__vo uint32_t EXTICR[4];		// SYSCFG external interrupt configuration registers
	__vo uint32_t CDGR2;			// SYSCFG configuration register 2
	__vo uint32_t SWPR;				// SYSCFG SRAM2 write protection register
	__vo uint32_t SKR;				// SYSCFG SRAM2 key register
	__vo uint32_t SWPR2; 			// SYSCFG SRAM2 write protection register 2
}SYSCFG_RegDef_t;


typedef struct{
	__vo uint16_t CR1;				// SPI control register 1
	uint16_t RESERVED1;				// Reserved memory not to be touched
	__vo uint16_t CR2;				// SPI control register 2
	uint16_t RESERVED2;				// Reserved memory not to be touched
	__vo uint16_t SR;				// SPI status register
	uint16_t RESERVED3;				// Reserved memory not to be touched
	__vo uint16_t DR;				// SPI data register
	uint16_t RESERVED4;				// Reserved memory not to be touched
	__vo uint16_t CRCPR;			// SPI CRC polynomial register
	uint16_t RESERVED5;				// Reserved memory not to be touched
	__vo uint16_t RXCRCR;			// SPI Rx CRC register
	uint16_t RESERVED6;				// Reserved memory not to be touched
	__vo uint16_t TXCRCR;			// SPI Tx CRC register
}SPI_RegDef_t;


typedef struct{
	__vo uint32_t CR1; 				// I2C control register 1
	__vo uint32_t CR2;				// I2C control register 2
	__vo uint32_t OAR1;				// I2C own address register 1
	__vo uint32_t OAR2;				// I2C own address register 2
	__vo uint32_t TIMINGR;			// I2C timing register
	__vo uint32_t TIMEOUTR;			// I2C timeout register
	__vo uint32_t ISR;				// I2C interrupt and status register
	__vo uint32_t ICR;				// I2C interrupt clear register
	__vo uint32_t PECR;				// I2C PEC (Packet Error Checking) register
	__vo uint32_t RXDR;				// I2C receive data register
	__vo uint32_t TXDR;				// I2C transmit data register
}I2C_RegDef_t;

// Register defintion for USARTx peripheral
typedef struct{
	uint32_t CR1;					// USART control register 1
	uint32_t CR2;					// USART control register 2
	uint32_t CR3;					// USART control register 3
	uint32_t BRR;					// USART baud Rate Register
	uint32_t GTPR;					// USART guard time and prescaler register
	uint32_t RTOR;					// USART receiver timeout register
	uint32_t RQR;					// USART request register
	uint32_t ISR;					// USART interrupt and status register
	uint32_t ICR;					// USART interrupt flag clear register
	uint32_t RDR;					// USART receive data register
	uint32_t TDR;					// USART transmit data register
}USART_RegDef_t;


#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC 	((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI 	((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG 	((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1 ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1 ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3 ((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1 	((USART_RegDef_t *)USART1_BASEADDR)
#define USART2 	((USART_RegDef_t *)USART2_BASEADDR)
#define USART3 	((USART_RegDef_t *)USART3_BASEADDR)
#define UART4 	((USART_RegDef_t *)UART4_BASEADDR)
#define UART5 	((USART_RegDef_t *)UART5_BASEADDR)


//---------------------------------Clock enable macros for different peripherals---------------------------------------------
// Clock enable macros for GPIOx peripherals

#define GPIOA_PCLK_EN() (RCC -> AHB2ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC -> AHB2ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC -> AHB2ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC -> AHB2ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC -> AHB2ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC -> AHB2ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC -> AHB2ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC -> AHB2ENR |= (1 << 7))


// Clock enable macros for SPIx peripherals

#define SPI1_PCLK_EN() (RCC -> APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC -> APB1ENR1 |= (1 << 14))
#define SPI3_PCLK_EN() (RCC -> APB1ENR1 |= (1 << 15))


// Clock enable macros for I2Cx peripherals

#define I2C1_PCLK_EN() (RCC -> APB1ENR1 |= (1 << 21))
#define I2C2_PCLK_EN() (RCC -> APB1ENR1 |= (1 << 22))
#define I2C3_PCLK_EN() (RCC -> APB1ENR1 |= (1 << 23))

// Clock enable macros for USARTx peripherals

#define USART1_PCLK_EN() (RCC -> APB2ENR |= (1 << 14))
#define USART2_PCLK_EN() (RCC -> APB1ENR1 |= (1 << 17))
#define USART3_PCLK_EN() (RCC -> APB1ENR1 |= (1 << 18))
#define UART4_PCLK_EN() (RCC -> APB1ENR1 |= (1 << 19))
#define UART5_PCLK_EN() (RCC -> APB1ENR1 |= (1 << 20))

// Clock enable macros for SYSCFGx peripheral

#define SYSCFG_PCLK_EN() (RCC -> APB2ENR |= (1 << 0))

//---------------------------------Clock disable macros for different peripherals---------------------------------------------
//Clock disable macros for GPIOx peripheraLS

#define GPIOA_PCLK_DI() (RCC -> AHB2ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC -> AHB2ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC -> AHB2ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC -> AHB2ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC -> AHB2ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC -> AHB2ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (RCC -> AHB2ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (RCC -> AHB2ENR &= ~(1 << 7))

// Clock disable macros for I2Cx peripherals

#define I2C1_PCLK_DI() (RCC -> APB1ENR1 &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC -> APB1ENR1 &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC -> APB1ENR1 &= ~(1 << 23))

// Clock disable macros for SPIx peripherals

#define SPI1_PCLK_DI() (RCC -> APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC -> APB1ENR1 &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC -> APB1ENR1 &= ~(1 << 15))

// Clock disable macros for USARTx peripherals

#define USART1_PCLK_DI() (RCC -> APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI() (RCC -> APB1ENR1 &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC -> APB1ENR1 &= ~(1 << 18))
#define UART4_PCLK_DI() (RCC -> APB1ENR1 &= ~(1 << 19))
#define UART5_PCLK_DI() (RCC -> APB1ENR1 &= ~(1 << 20))

// Clock disable macros for SYSCFGx peripheral

#define SYSCFG_PCLK_DI() (RCC -> APB2ENR &= ~(1 << 0))



//---------------------------------Macros to reset GPIOs---------------------------------------------
#define GPIOA_REG_RESET() 			do{(RCC -> AHB2RSTR |= (1 << 0));	(RCC -> AHB2RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET() 			do{(RCC -> AHB2RSTR |= (1 << 1));	(RCC -> AHB2RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET() 			do{(RCC -> AHB2RSTR |= (1 << 2));	(RCC -> AHB2RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET() 			do{(RCC -> AHB2RSTR |= (1 << 3));	(RCC -> AHB2RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET() 			do{(RCC -> AHB2RSTR |= (1 << 4));	(RCC -> AHB2RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET() 			do{(RCC -> AHB2RSTR |= (1 << 5));	(RCC -> AHB2RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()			do{(RCC -> AHB2RSTR |= (1 << 6));	(RCC -> AHB2RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET() 			do{(RCC -> AHB2RSTR |= (1 << 7));	(RCC -> AHB2RSTR &= ~(1 << 7)); }while(0)

//  Returns a number 0 to 7 for a given GPIO base address (x)

#define GPIO_BASEADDR_TO_CODE(x)	(	(x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOF) ? 5 :\
										(x == GPIOG) ? 6 : 0)

// Generic macros

#define ENABLE				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET 		SET
#define GPIO_PIN_RESET 		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET

// Interrupt request numbers of ST32" L4xx

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40
#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73
#define IRQ_NO_I2C4_EV		83
#define IRQ_NO_I2C4_ER 		84

// Macros for all possible priority levels

#define NVIC_IRQ_PRI0 		0
#define NVIC_IRQ_PRI1 		1
#define NVIC_IRQ_PRI2 		2
#define NVIC_IRQ_PRI3 		3
#define NVIC_IRQ_PRI4 		4
#define NVIC_IRQ_PRI5 		5
#define NVIC_IRQ_PRI6 		6
#define NVIC_IRQ_PRI7 		7
#define NVIC_IRQ_PRI8 		8
#define NVIC_IRQ_PRI9 		9
#define NVIC_IRQ_PRI10 		10
#define NVIC_IRQ_PRI11 		11
#define NVIC_IRQ_PRI12 		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14 		14
#define NVIC_IRQ_PRI15 		15




#define MSI_RANGE_0 		100000U
#define MSI_RANGE_1			200000U
#define MSI_RANGE_2			400000U
#define MSI_RANGE_3			800000U
#define MSI_RANGE_4			1000000U
#define MSI_RANGE_5			2000000U
#define MSI_RANGE_6			4000000U
#define MSI_RANGE_7			8000000U
#define MSI_RANGE_8			16000000U
#define MSI_RANGE_9			24000000U
#define MSI_RANGE_10		32000000U
#define MSI_RANGE_11		48000000U

#define HSI16_Clock_Frequency 	16000000


//---------------------------------MACROS FOR RCC BIT POSITIONS---------------------------------------------
#define RCC_CR_MSION			0
#define RCC_CR_MSIRDY			1
#define RCC_CR_MSIIPLLEN		2
#define RCC_CR_MSIRGSEL			3
#define RCC_CR_MSIRANGE			4
#define RCC_CR_HSION			8
#define RCC_CR_HSIKERON			9
#define RCC_CR_HSIRDY			10
#define RCC_CR_HSIASFS			11
#define RCC_CR_HSEON			16
#define RCC_CR_HSERDY			17
#define RCC_CR_HSEBYP			18
#define RCC_CR_CSSON			19
#define RCC_CR_PLLON			24
#define RCC_CR_PLLRDY			25
#define RCC_CR_PLLSAI1ON		26
#define RCC_CR_PLLSAI1RDY		27
#define RCC_CR_PLLSAI2ON		28
#define RCC_CR_PLLSAI2RDY		29

#define RCC_CCIPR_USART1SEL		0
#define RCC_CCIPR_USART2SEL		2
#define RCC_CCIPR_USART3SEL		4
#define RCC_CCIPR_USART4SEL		6
#define RCC_CCIPR_USART5SEL		8
#define RCC_CCIPR_LPUART1SEL	10
#define RCC_CCIPR_I2C1SEL		12
#define RCC_CCIPR_I2C2SEL		14
#define RCC_CCIPR_I2C3SEL		16
#define RCC_CCIPR_LPTIM1SEL		18
#define RCC_CCIPR_LPTIM2SEL		20
#define RCC_CCIPR_SAI1SEL		22
#define RCC_CCIPR_SAI2SEL		24
#define RCC_CCIPR_CLK48SEL		26
#define RCC_CCIPR_ADCSEL		28
#define RCC_CCIPR_SWPMI1SE		30
#define RCC_CCIPR_DFSDM1SEL		31

//---------------------------------MACROS FOR SPI BIT POSITIONS---------------------------------------------
#define SPIx_CR1_CPHA			0  	// Clock phase
#define SPIx_CR1_CPOL           1  	// Clock polarity
#define SPIx_CR1_MSTR           2  	// Master selection
#define SPIx_CR1_BR             3  	// Baud rate control
#define SPIx_CR1_SPE            6  	// SPI enable
#define SPIx_CR1_LSBFIRST      	7  	// Frame format
#define SPIx_CR1_SSI            8  	// Internal slave select
#define SPIx_CR1_SSM            9  	// Software slave managemen
#define SPIx_CR1_RX_ONLY        10 	// Receive only mode enabled
#define SPIx_CR1_CRCL           11 	// CRC length
#define SPIx_CR1_CRCNEXT       	12 	// Transmit CRC next
#define SPIx_CR1_CRCEN         	13 	// Hardware CRC calculation enable
#define SPIx_CR1_BIDIOE         14 	// Output enable in bidirectional mode
#define SPIx_CR1_BIDI_MODE      15 	// Bidirectional data mode enable

#define SPIx_CR2_RXDMAEN		0	// Rx buffer DMA enable
#define SPIx_CR2_TXDMAEN		1   // Tx buffer DMA enable
#define SPIx_CR2_SSOE			2   // SS output enable
#define SPIx_CR2_NSSP			3   // NSS pulse management
#define SPIx_CR2_FRF			4   // Frame format
#define SPIx_CR2_ERRIE			5   // Error interrupt enable
#define SPIx_CR2_RXNEIE			6   // RX buffer not empty interrupt enable
#define SPIx_CR2_TXEIE			7   // Tx buffer empty interrupt enable
#define SPIx_CR2_DS				8   // Data size
#define SPIx_CR2_FRXTH			12  // FIFO reception threshold
#define SPIx_CR2_LDMA_RX		13  // Last DMA transfer for reception
#define SPIx_CR2_LDMA_TX		14  // Last DMA transfer for transmission

#define SPIx_SR_RXNE			0 	// Receive buffer not empty
#define SPIx_SR_TXE				1 	// Transmit buffer empty
#define SPIx_SR_CRCE_RR			4 	// CRC error flag
#define SPIx_SR_MODF			5	// Mode fault
#define SPIx_SR_OVR				6	// Overrun flag
#define SPIx_SR_BSY				7	// Busy flag
#define SPIx_SR_FRE				8	// Frame format error
#define SPIx_SR_FRLVL			9	// FIFO reception level
#define SPIx_SR_FTLVL			11	// FIFO transmission level


//---------------------------------MACROS FOR I2C REGISTER OFFSETS-----------------------------------------
// These are neccessary because the I2C interrupt handlers are defined per I2C (i.e I2C1, I2C2, I2C3, I2C4)
// and the interrupt functions have to have void parameters
#define I2Cx_CR					0x0
#define I2Cx_CR2				0x4
#define I2Cx_OAR1				0x8
#define I2Cx_OAR2				0xC
#define I2Cx_TIMINGR			0x10
#define I2Cx_TIMEOUTR			0x14
#define I2Cx_ISR				0x18
#define I2Cx_ICR				0x1C
#define I2Cx_PECR				0x20
#define I2Cx_RXDR				0x24
#define I2Cx_TXDR				0x28

//---------------------------------MACROS FOR I2C BIT POSITIONS---------------------------------------------
#define I2Cx_CR1_PE				0 	// Peripheral enable
#define I2Cx_CR1_TXIE 			1	// TX interrupt enable
#define I2Cx_CR1_RXIE			2	// RX interrupt enable
#define I2Cx_CR1_ADDRIE			3	// Address match interrupt enable (slave only)
#define I2Cx_CR1_NACKIE			4	// Not acknowledge received interrupt enable
#define I2Cx_CR1_STOPIE			5	// Stop detection interrupt enab;e
#define I2Cx_CR1_TCIE			6	// Transfer complete interrupt enable
#define I2Cx_CR1_ERRIE			7	// Error interrupts enable
#define I2Cx_CR1_DNF			8 	// Digital noise filter
#define I2Cx_CR1_ANFOFF			12	// Analog noise filter OFF
#define I2Cx_CR1_TXDMAEN		14  // DMA transmission requests enable
#define I2Cx_CR1_RXDAMEN		15	// DMA receptin requests enable
#define I2Cx_CR1_SBC 			16	// Slave byte control
#define I2Cx_CR1_NOSTRECH		17	// Clock streching disable
#define I2Cx_CR1_WUPEN			18	// Wakeup from Stop mode enable
#define I2Cx_CR1_GCEN			19	// General call enable
#define I2Cx_CR1_SMBHEN			20	// SMBus host address enable
#define I2Cx_CR1_SMBDEN			21	// SMBus device default address enable
#define I2Cx_CR1_ALERTEN		22 	// SMBus alert enable
#define I2Cx_CR1_PECEN			23	// PEC enable

#define I2Cx_CR2_SADD 			0	// Slave address
#define I2Cx_CR2_RD_WRN			10	// Transfer direction (master mode)
#define I2Cx_CR2_ADD10			11	// 10-bit addresssing mode (master mode)
#define I2Cx_CR2_HEAD10R		12 	// 10-bit address header only read direction (master receiver mode)
#define I2Cx_CR2_START			13	// Start generation
#define I2Cx_CR2_STOP			14  // Stop generation
#define I2Cx_CR2_NACK			15	// NACK generation
#define I2Cx_CR2_NBYTES			16	// Number of bytes
#define I2Cx_CR2_RELOAD			24	// NBYTES reload mode
#define I2Cx_CR2_AUTOEND		25	// Automatic end mode (master mode)
#define I2Cx_CR2_PECBYTE		26	// Packet error checking byte

#define I2Cx_OAR1_OA1			0   // Interface own slave address
#define I2Cx_OAR1_OA1MODE		10	// Own Address 1 10-bit mode
#define I2Cx_OAR1_OA1EN			15  // Own Address 1 enable

#define I2Cx_OAR2_OA2			1 	// Interface address
#define I2Cx_OAR2_OA2MSK		8	// Own Address 2 masks
#define I2Cx_OAR2_OA2EN			15	// Own Address 2 enable

#define I2Cx_TIMINGR_SCLL		0	// SCL low period (master mode)
#define I2Cx_TIMINGR_SCLH		8	// SCL high period (master mode)
#define I2Cx_TIMINGR_SDADEL		16	// Data hold time
#define I2Cx_TIMINGR_SCLDEL		20	// Data setup time
#define I2Cx_TIMINGR_PRESC		28	// Timing prescaler

#define I2Cx_ISR_TXE			0	// Transmit data reigster empty (transmitters)
#define I2Cx_ISR_TXIS			1	// Transmit interrupt status (transmitters)
#define I2Cx_ISR_RXNE			2	// Receive data register not empty (receivers)
#define I2Cx_ISR_ADDR			3	// Address matched (slave mode)
#define I2Cx_ISR_NACKF			4	// Not Acknowledge received flag
#define I2Cx_ISR_STOPF			5	// Stop detection flag
#define I2Cx_ISR_TC				6	// Transfer complete (master mode)
#define I2Cx_ISR_TCR			7	// Transfer complete reload
#define I2Cx_ISR_BERR			8	// Bus error
#define I2Cx_ISR_ARLO			9	// Arbitration lost
#define I2Cx_ISR_OVR 			10	// Overrun/Underrun (slave mode)
#define I2Cx_ISR_PECERR			11	// PEC Error in reception
#define I2Cx_ISR_TIMEOUT		12	// Timeout or t_LOW detection flag
#define I2Cx_ISR_ALERT			13	// SMBus alert
#define I2Cx_ISR_BUSY			15	// Bus busy
#define I2Cx_ISR_DIR			16	// Transfer direction (slave mode)
#define I2Cx_ISR_ADDCODE		17	// Address match code (slave mode)

#define I2Cx_ICR_ADDRCF			3	// Address matched flag clear
#define I2Cx_ICR_NACKCF			4	// Not acknowledge flag clear
#define I2Cx_ICR_STOPCF			5	// STOP detection flag clear
#define I2Cx_ICR_BERRCF			8	// Bus error flag clear
#define I2Cx_ICR_ARLOCF			9	// Arbitration lost flag clear
#define I2Cx_ICR_OVRCF			10	// Overrun/Underrun flag clear
#define I2Cx_ICR_PECCF			11  // PEC error flag clear
#define I2Cx_ICR_TIMOUTCF		12	// Timeout detection flag clear

//---------------------------------MACROS FOR UART/USART BIT POSITIONS---------------------------------------------
#define USARTx_CR1_UE			0 	// USART enable
#define USARTx_CR1_UESM			1   // USART enable in Stop mode
#define USARTx_CR1_RE			2   // Reciever enable
#define USARTx_CR1_TE			3   // Transmitter enable
#define USARTx_CR1_IDLEIE		4   // IDLE interrupt enable
#define USARTx_CR1_RXNEIE		5   // RXNE interrupt enable
#define USARTx_CR1_TXEIE		6   // Transmission complete interrupt enable
#define USARTx_CR1_TCIE			7   // Interrupt enable
#define USARTx_CR1_PEIE			8   // Interrupt enable
#define USARTx_CR1_PS			9   // Parity selection
#define USARTx_CR1_PCE			10  // Parity control enable
#define USARTx_CR1_WAKE			11  // Receiver wakeup method
#define USARTx_CR1_M0			12  // World length
#define USARTx_CR1_MME			13  // Mute mode enable
#define USARTx_CR1_CMIE			14  // Character match interrupt enable
#define USARTx_CR1_OVER8		15  // Oversampling mode
#define USARTx_CR1_DEDT0		16  // Driver enable de-assertion time
#define USARTx_CR1_DEDT1		17  // Driver enable de-assertion time
#define USARTx_CR1_DEDT2		18  // Driver enable de-assertion time
#define USARTx_CR1_DEDT3		19  // Driver enable de-assertion time
#define USARTx_CR1_DEDT4		20  // Driver enable de-assertion time
#define USARTx_CR1_DEAT0		21  // Driver enable assertion time
#define USARTx_CR1_DEAT1		22  // Driver enable assertion time
#define USARTx_CR1_DEAT2		23  // Driver enable assertion time
#define USARTx_CR1_DEAT3		24  // Driver enable assertion time
#define USARTx_CR1_DEAT4		15  // Driver enable assertion time
#define USARTx_CR1_RTOIE		16  // Receiver timeout interrupt enable
#define USARTx_CR1_EOBIE		27  // End of block interrupt software
#define USARTx_CR1_M1			28  // Word length

#define USARTx_CR2_ADDM7		4   // 7-bit address detection/4-bit address detection
#define USARTx_CR2_LBL			5   // LIN break detection length
#define USARTx_CR2_LBDIE		6   // LIN break detection interrupt enable
#define USARTx_CR2_LBCL			8   // Last bit clock pulse
#define USARTx_CR2_CPHA			9   // Clock phase
#define USARTx_CR2_CPOL			10  // Clock polarity
#define USARTx_CR2_CLKEN		11  // Clock enable
#define USARTx_CR2_STOP			12  // STOP bits
#define USARTx_CR2_LINEN		14  // LIN mode enable
#define USARTx_CR2_SWAP			15  // Swap TX/RX pins
#define USARTx_CR2_RXINV		16  // RX pin active lebel inversion
#define USARTx_CR2_TXINV		17  // TX pin active level inversion
#define USARTx_CR2_DATAINV		18  // Binary data inversion
#define USARTx_CR2_MSBFIRST		19  // Most significatn bit first
#define USARTx_CR2_ABREN		20  // Auto baud rate enable
#define USARTx_CR2_ABRMOD0		21  // Auto baud rate mode
#define USARTx_CR2_ABRMOD1		22  // Auto baud rate mode
#define USARTx_CR2_RTOEN		23  // Receiver timeout enable
#define USARTx_CR2_ADD			24  // Address of the USART node

#define USARTx_CR3_EIE			0   // Error interrupt enable
#define USARTx_CR3_IREN			1   // IrDA mode enable
#define USARTx_CR3_IRLP			2   // IrDA low-power
#define USARTx_CR3_HDSEL		3   // Half-duplex selection
#define USARTx_CR3_NACK			4   // Smartcard NACK enable
#define USARTx_CR3_SCEN			5   // Smartcard mode enable
#define USARTx_CR3_DMAR			6   // DMA enable receiver
#define USARTx_CR3_DMAT			7   // DMA enable transmitter
#define USARTx_CR3_RTSE			8   // RTS enable
#define USARTx_CR3_CTSE			9   // CTS enable
#define USARTx_CR3_CTSIE		10  // CTS interrupt enable
#define USARTx_CR3_ONEBIT		11  // One sample bit method enable
#define USARTx_CR3_OVRDIS		12  // Overrun disable
#define USARTx_CR3_DDRE			13  // DMA disable on reception error
#define USARTx_CR3_DEM			14  // Driver enable mode
#define USARTx_CR3_DEP			15  // Driver enable polairty selection
#define USARTx_CR3_SCARNT		17  // Smartcard auto-retry count
#define USARTx_CR3_WUS			20  // Wakeup from stop mode interrupt flag selection
#define USARTx_CR3_WUFIE		22  // Wakeup from stop mode interrupt enable
#define USARTx_CR3_UCESM		23  // USART clock enable in stop mode

#define USARTx_BRR_BRR			0   // Baud rate reigster

#define USARTx_GTPR_PSC			0   // Prescaler value
#define USARTx_GTPR_GT			7   // Guard time value

#define USARTx_RTOR_RTO			0   // Receiver timeout value
#define USARTx_GTPR_BLEN		23  // Bock length

#define USARTx_RQR_ABRRQ		0   // Auto baud rate request
#define USARTx_RQR_SBKRQ		1   // Send break request
#define USARTx_RQR_MMRQ			2   // Mute mode request
#define USARTx_RQR_RXFRQ		3   // Recieve data flush request
#define USARTx_RQR_TXFRQ		4   // Transmit data flush request

#define USARTx_ISR_PE			0   // Parity error
#define USARTx_ISR_FE			1   // Framing error
#define USARTx_ISR_NF			2   // START bit noise deteciton flag
#define USARTx_ISR_ORE			3   // Overrun error
#define USARTx_ISR_IDLE			4   // Idle line deteceted
#define USARTx_ISR_RXNE			5   // Read data register not empty
#define USARTx_ISR_TC			6   // Transmission complete
#define USARTx_ISR_TXE			7   // Transmit data register empty
#define USARTx_ISR_LBDF			8   // LIN break detection flag
#define USARTx_ISR_CTSIF		9   // CTS interrupt flag
#define USARTx_ISR_CTS			10  // CTS flag
#define USARTx_ISR_RTOF			11  // Receiver timeout
#define USARTx_ISR_EOBF			12  // End of block flag
#define USARTx_ISR_ABRE			14  // Auto baud rate error
#define USARTx_ISR_ABRF			15  // Auto baud rate flag
#define USARTx_ISR_BUST			16  // Busy flag
#define USARTx_ISR_CMF			17  // Character match flag
#define USARTx_ISR_SBKF			18  // Send breakflag
#define USARTx_ISR_RWU			19  // Receive wakeup from mute mode
#define USARTx_ISR_WUF			20  // Wakeup from stop mode flag
#define USARTx_ISR_TEACK		21  // Transmit enable acknowledge flag
#define USARTx_ISR_REACK		22  // Receive enable acknowledge flag

#define USARTx_ICR_PECF			0   // Parity error clear flag
#define USARTx_ICR_FECF			1   // Framing error clear flag
#define USARTx_ICR_NCF			2   // Noise deteceted clear flag
#define USARTx_ICR_ORECF		3   // Overrun error clear flag
#define USARTx_ICR_IDLECF		4   // Idle line detected clear flag
#define USARTx_ICR_TCCF			6   // Transmission complete clear flag
#define USARTx_ICR_LBDCF		8   // LIN break detection clear glag
#define USARTx_ICR_CTSCF		9   // CTS clear glag
#define USARTx_ICR_RTOCF		11  // Receiver imeout clear flag
#define USARTx_ICR_EOBCF		12  // End of block clear flag
#define USARTx_ICR_CMCF			17  // Character match clear flag
#define USARTx_ICR_WUCF			20  // Wakeup from stop mode clear flag

#define USARTx_RDR_RDR			0   // Receive data value

#define USARTx_TDR_TDR			0   // Transmit data value

#include "stm32l4xx_gpio_driver.h"
#include "stm32l4xx_spi_driver.h"
#include "stm32l4xx_i2c_driver.h"
#include "stm32l4xx_USART_driver.h"
#include "stm32l4xx_rcc_driver.h"



#endif /* INC_STM32L4XX_H_ */
