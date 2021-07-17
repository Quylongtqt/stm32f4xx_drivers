/*
 * stm32f407xx.h
 *
 *  Created on: Jun 25, 2021
 *      Author: Quy Long
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdio.h>


/********************** START : Processor Specific Deatils  ************************/
/*
 * ARM Cotex Mx Processor NVIC ISERx registers Address
 *
 */

// Interrupt Set-enable Registers
#define NVIC_ISER0		( (volatile uint32_t*)0xE000E100 )
#define NVIC_ISER1		( (volatile uint32_t*)0xE000E104 )
#define NVIC_ISER2		( (volatile uint32_t*)0xE000E108 )
#define NVIC_ISER3		( (volatile uint32_t*)0xE000E10C )
#define NVIC_ISER4		( (volatile uint32_t*)0xE000E110 )
#define NVIC_ISER5		( (volatile uint32_t*)0xE000E114 )
#define NVIC_ISER6		( (volatile uint32_t*)0xE000E118 )
#define NVIC_ISER7		( (volatile uint32_t*)0xE000E11C )
 // Interrupt Clear-enable Registers
#define NVIC_ICER0		( (volatile uint32_t*)0xE000E180 )
#define NVIC_ICER1		( (volatile uint32_t*)0xE000E184 )
#define NVIC_ICER2		( (volatile uint32_t*)0xE000E188 )
#define NVIC_ICER3		( (volatile uint32_t*)0xE000E18C )
#define NVIC_ICER4		( (volatile uint32_t*)0xE000E190 )
#define NVIC_ICER5		( (volatile uint32_t*)0xE000E194 )
#define NVIC_ICER6		( (volatile uint32_t*)0xE000E198 )
#define NVIC_ICER7		( (volatile uint32_t*)0xE000E19C )

/*
 * ARM Cotex Mx Processor priority registers Address calculation
 *
 */

#define NVIC_PR_BASEADDR	( (volatile uint32_t*)0xE000E400 )

/*
 * ARM Cotex Mx Processor Number of the priority bits implemented in Priorities Register
 *
 */

#define NO_PR_BITS_IMPLEMENTED		4

/*
 * Base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR				0x08000000U			/* <explain MACRO here> */
#define SRAM1_BASEADDR				0x20000000U			/* size 112KB-> 112x1024 = 114688 = 0x1C000 */
#define SRAM2_BASEADDR				0x2001C000U
#define SRAM						SRAM1_BASEADDR
#define ROM							0x1FFF0000U			/* System Memory */

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR				0x40000000U			/* <explain MACRO here> */
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASEADDR			0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1
 */

#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)
/*
 * Base addresses of peripherals which are hanging on APB1
 */

#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)

#define CAN1_BASEADDR				(APB1PERIPH_BASEADDR + 0x6400)
#define CAN2_BASEADDR				(APB1PERIPH_BASEADDR + 0x6800)

/*
 * Base addresses of peripherals which are hanging on APB2
 */

#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x8800)
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)

/********************** peripheral register definition structures ************************/
/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of registers of SPI peripheral of SMT32F4xx family of MCUs may be different
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 */

/*
 * peripheral register definition structure for GPIO
 */
typedef struct
{
	volatile uint32_t MODER;		/* GPIO port mode register,					Address offset	0x00 */
	volatile uint32_t OTYPER;		/* GPIO port output type register , 		Address offset	0x04 */
	volatile uint32_t OSPEEDER;		/* GPIO port output speed register, 		Address offset	0x08 */
	volatile uint32_t PUPDR;		/* GPIO port pull-up/pull-down register, 	Address offset	0x0C */
	volatile uint32_t IRD;			/* GPIO port input data register, 			Address offset	0x10 */
	volatile uint32_t ODR;			/* GPIO port output data register, 			Address offset	0x14 */
	volatile uint32_t BSRR;			/* GPIO port bit set/reset register, 		Address offset	0x18 */
	volatile uint32_t LCKR;			/* GPIO port configuration lock register, 	Address offset	0x1C */
	volatile uint32_t AFR[2];		/* GPIO alternate function register 		Address offset	0x20 */

}GPIO_RegDef_t;

/*
 * peripheral register definition structure for CRC
 */
typedef struct
{
	volatile uint32_t CR;			
	volatile uint32_t PLLCFGR;		
	volatile uint32_t CFGR;			
	volatile uint32_t CIR;			
	volatile uint32_t AHB1RSTR;		
	volatile uint32_t AHB2RSTR;		
	volatile uint32_t AHB3RSTR;		
			 uint32_t RESERVED0;	
	volatile uint32_t APB1RSTR;		
	volatile uint32_t APB2RSTR;		
			 uint32_t RESERVED1[2];	
	volatile uint32_t AHB1ENR;		
	volatile uint32_t AHB2ENR;		
	volatile uint32_t AHB3ENR;		
	volatile uint32_t RESERVED2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
			 uint32_t RESERVED3[2];	
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
			 uint32_t RESERVED4;
			 uint32_t RESERVED5[2];
	volatile uint32_t APB2LPENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
			 uint32_t RESERVED6[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;

}RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	volatile uint32_t IMR; 		/* Interrupt mask register, Address offset	0x00 */
	volatile uint32_t EMR; 		/* Event mask register, Address offset	0x04 */
	volatile uint32_t RTSR; 	/* Rising trigger selection register, Address offset	0x08 */
	volatile uint32_t FTSR; 	/* Falling trigger selection register, Address offset	0x0C */
	volatile uint32_t SWIER; 	/* Software interrupt event register, Address offset	0x10 */
	volatile uint32_t PR; 		/* Pending register, Address offset	0x14 */

}EXTI_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	volatile uint32_t MEMRM; 		/* SYSCFG memory remap register, Address offset	0x00 */
	volatile uint32_t PMC; 			/* SYSCFG peripheral mode configuration register, Address offset	0x04 */
	volatile uint32_t EXTICR[4]; 	/* SYSCFG peripheral mode configuration register, Address offset	0x08 - 0x14 */
			 uint32_t RESERVED1[2];	/* Reserved, Address offset	0x08 - 0x1C */
	volatile uint32_t CMPCR; 		/* Compensation cell control register, Address offset	0x20 */
			 uint32_t RESERVED2[2];	/* Reserved, Address offset	0x24 - 0x28 */
	volatile uint32_t CFGR; 		/* TODO , Address offset	0x2C */

}SYSCFG_RegDef_t;

/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	volatile uint32_t CR1;		/* SPI control register,			Address offset	0x00 */
	volatile uint32_t CR2;		/* SPI control register, 			Address offset	0x04 */
	volatile uint32_t SR;		/* SPI status register, 			Address offset	0x08 */
	volatile uint32_t DR;		/* SPI data register, 				Address offset	0x0C */
	volatile uint32_t CRCPR;	/* SPI CRC polynomial register, 	Address offset	0x10 */
	volatile uint32_t RXCRCR;	/* SPI RX CRC register, 			Address offset	0x14 */
	volatile uint32_t TXCRCR;	/* SPI TX CRC register, 			Address offset	0x18 */
	volatile uint32_t I2SCFGR;	/* SPI_I2S configuration register, 	Address offset	0x1C */
	volatile uint32_t I2SPR;	/* SPI_I2S prescaler register,	 	Address offset	0x20 */

}SPI_RegDef_t;

/*
 * peripheral definitions
 */

#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()		( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()		( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()		( RCC->AHB1ENR |= (1 << 8) )

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()		( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= (1 << 23) )

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()		( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()		( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= (1 << 15) )

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()		( RCC->APB2ENR |= (1 <<  4) )
#define USART2_PCLK_EN()		( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()		( RCC->APB1ENR |= (1 << 18) )
#define USART4_PCLK_EN()		( RCC->APB2ENR |= (1 << 19) )
#define USART5_PCLK_EN()		( RCC->APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN()		( RCC->APB2ENR |= (1 <<  5) )
/*
 * Clock Enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()		( RCC->APB2ENR |= (1 << 14) )

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()		( RCC->AHB1DIR &= ~ (1 << 0) )
#define GPIOB_PCLK_DI()		( RCC->AHB1DIR &= ~ (1 << 1) )

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()		( RCC->APB1DIR &= ~ (1 << 21) )
#define I2C2_PCLK_DI()		( RCC->APB1DIR &= ~ (1 << 22) )
#define I2C3_PCLK_DI()		( RCC->APB1DIR &= ~ (1 << 23) )

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()		( RCC->APB2DIR &= ~ (1 << 12) )
#define SPI2_PCLK_DI()		( RCC->APB1DIR &= ~ (1 << 14) )
#define SPI3_PCLK_DI()		( RCC->APB1DIR &= ~ (1 << 15) )

/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DI()		( RCC->APB2DIR &= ~ (1 <<  4) )
#define USART2_PCLK_DI()		( RCC->APB1DIR &= ~ (1 << 17) )
#define USART3_PCLK_DI()		( RCC->APB1DIR &= ~ (1 << 18) )
#define USART4_PCLK_DI()		( RCC->APB2DIR &= ~ (1 << 19) )
#define USART5_PCLK_DI()		( RCC->APB1DIR &= ~ (1 << 20) )
#define USART6_PCLK_DI()		( RCC->APB2DIR &= ~ (1 <<  5) )
/*
 * Clock Disable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()		( RCC->APB2DIR &= ~ (1 << 14) )

/*
 * Reset GPIOx peripheral Macros
 */

#define GPIOA_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));}while(0)

/*
 * Reset SPIx peripheral Macros
 */

#define SPI1_REG_RESET()		do{(RCC->AHB2RSTR |= (1 << 12)); (RCC->AHB2RSTR &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));}while(0)

/*
 * EXTIx configuration
 * This macro returns a code (0 to 7) for a given GPIO base address(x)
 */

#define GPIO_BASEADDR_TO_CODE(x)	(	(x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOF) ? 5 :\
										(x == GPIOG) ? 6 :\
										(x == GPIOH) ? 7 :8)

/*
 * IRQ (Interrupt ReQuest) number of STM32F407xx MCU
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/*
 * Macros for all the possible priority levels
 */
#define NVIC_IRQ_PRIO0		0
#define NVIC_IRQ_PRIO15		15

/*
 * Generic Macros
 */

#define	ENABLE				1
#define DISABLE				0
#define	SET 				ENABLE
#define RESET				DISABLE
#define	GPIO_PIN_SET 		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_SET			SET
#define FLAG_RESET			RESET
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_TBSY			7
#define SPI_SR_TIFRFE		8

#endif /* INC_STM32F407XX_H_ */
