/*
 * stm32f407xx.h
 *
 *  Created on: Jun 25, 2021
 *      Author: Quy Long
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>

/*
 * base address of Flash and SRAM address
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

typedef struct
{
	volatile uint32_t MODER;		/* GPIO port mode register, Address offset	0x00 */
	volatile uint32_t OTYPER;		/* GPIO port output type register , Address offset	0x04 */
	volatile uint32_t OSPEEDER;		/* GPIO port output speed register, Address offset	0x08 */
	volatile uint32_t PUPDR;		/* GPIO port pull-up/pull-down register, Address offset	0x0C */
	volatile uint32_t IRD;			/* GPIO port input data register, Address offset	0x10 */
	volatile uint32_t ODR;			/* GPIO port output data register, Address offset	0x14 */
	volatile uint32_t BSRR;			/* GPIO port bit set/reset register, Address offset	0x18 */
	volatile uint32_t LCKR;			/* GPIO port configuration lock register, Address offset	0x1C */
	volatile uint32_t AFR[2];		/* GPIO alternate function register */


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

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 0) )
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 1) )
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 2) )
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 3) )
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 4) )
#define GPIOF_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 5) )
#define GPIOG_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 6) )
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 7) )
#define GPIOI_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 8) )

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 21) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 22) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 23) )

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 12) )
#define SPI2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 14) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 15) )

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 4) )
#define USART2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 17) )
#define USART3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 18) )
#define USART4_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 19) )
#define USART5_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 20) )
#define USART6_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 5) )
/*
 * Clock Enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 14) )

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()		( RCC->AHB1DIR &= ~ ( 1 << 0) )
#define GPIOB_PCLK_DI()		( RCC->AHB1DIR &= ~ ( 1 << 1) )

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()		( RCC->APB1DIR &= ~ ( 1 << 21) )
#define I2C2_PCLK_DI()		( RCC->APB1DIR &= ~ ( 1 << 22) )
#define I2C3_PCLK_DI()		( RCC->APB1DIR &= ~ ( 1 << 23) )

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()		( RCC->APB2DIR &= ~ ( 1 << 12) )
#define SPI2_PCLK_DI()		( RCC->APB1DIR &= ~ ( 1 << 14) )
#define SPI3_PCLK_DI()		( RCC->APB1DIR &= ~ ( 1 << 15) )

/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DI()		( RCC->APB2DIR &= ~ ( 1 << 4) )
#define USART2_PCLK_DI()		( RCC->APB1DIR &= ~ ( 1 << 17) )
#define USART3_PCLK_DI()		( RCC->APB1DIR &= ~ ( 1 << 18) )
#define USART4_PCLK_DI()		( RCC->APB2DIR &= ~ ( 1 << 19) )
#define USART5_PCLK_DI()		( RCC->APB1DIR &= ~ ( 1 << 20) )
#define USART6_PCLK_DI()		( RCC->APB2DIR &= ~ ( 1 << 5) )
/*
 * Clock Disable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()		( RCC->APB2DIR &= ~ ( 1 << 14) )

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
 * Generic Macros
 */

#define	ENABLE	1
#define DISABLE	0
#define	SET 	ENABLE
#define RESET	DISABLE
#define	GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET	RESET


#endif /* INC_STM32F407XX_H_ */
