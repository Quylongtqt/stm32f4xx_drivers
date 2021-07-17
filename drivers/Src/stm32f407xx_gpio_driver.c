/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jun 28, 2021
 *      Author: Quy Long
 */
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
/****************************************************************************************************
 * 								APIs supported by this driver
 *			 For more information about APIs check the function definitions
 *
 ****************************************************************************************************/

/****************************************************************************************************
  * @function		-	GPIO_PeriperalClockControl
  *
  * @brief			-	This Function enables or disables peripheral clock for the given GPIO port
  *
  * @param[in]    		-	pGPIOx : Based address of the GPIO peripheral
  * @param[in]  		-	ENorDIS: ENABLE or DISABLE
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
void GPIO_PeriperalClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDIS)
{
	if(ENorDIS == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}else
	{
		//Todo
	}
}

/****************************************************************************************************
  * @function		-	GPIO_Init
  *
  * @brief			-	This Function
  *
  * @param[in]    		-	pGPIOHanlde : GPIOx
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
void GPIO_Init(GPIO_Handle_t *pGPIOHanlde)
{
	uint32_t temp = 0;	//temp. register

	// Enable the peripheral clock
	GPIO_PeriperalClockControl(pGPIOHanlde->pGPIOx, ENABLE);

	/* 1. Configure the mode of gpio pin */
	if(pGPIOHanlde->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// the non interrupt mode
		temp = (pGPIOHanlde->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHanlde->pGPIOx->MODER &= ~( 0x3 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber); // Clearing
		pGPIOHanlde->pGPIOx->MODER |= temp;
	}else
	{
		// the interrupt mode
		if(pGPIOHanlde->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. Configure the FTSR
			// Enable FTSR
			EXTI->FTSR |= ( 1 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber);
			// Disable RTSR
			EXTI->RTSR &= ~( 1 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber);
		}else if (pGPIOHanlde->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. Configure the RTSR
			// Disable FTSR
			EXTI->FTSR &= ~( 1 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber);
			// Enable RTSR
			EXTI->RTSR |= ( 1 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHanlde->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. Configure both FTSR and RTSR
			// Enable FTSR
			EXTI->FTSR |= ( 1 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber);
			// Enable RTSR
			EXTI->RTSR |= ( 1 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t EXTICRIndex = pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber / 4; //EXTICR Index like as EXTICR1/2/34
		uint8_t EXTIndex = pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber % 4; // EXT Index
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHanlde->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[EXTICRIndex] = portcode << (EXTIndex *4);

		//3. Enable the exti interrupt delivery using IMR ( Interrupt Mode Register)
		EXTI->IMR |= 1 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber;
	}
	temp = 0;

	/* 2. Configure the speed */
	temp = (pGPIOHanlde->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHanlde->pGPIOx->OSPEEDER &= ~( 0x3 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber); // Clearing
	pGPIOHanlde->pGPIOx->OSPEEDER |= temp;

	temp = 0;
	/* 3. Configure the PuPd settings */
	temp = (pGPIOHanlde->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHanlde->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber); // Clearing
	pGPIOHanlde->pGPIOx->PUPDR |= temp;

	temp = 0;
	/* 4. Configure the OpType */
	temp = (pGPIOHanlde->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHanlde->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber); // Clearing
	pGPIOHanlde->pGPIOx->OTYPER |= temp;

	/* 5. Configure the Alt Functionality */
	if(pGPIOHanlde->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
	// Configure the alt function registers
	uint8_t temp1, temp2;

	temp1 = pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber / 8;
	temp2 = pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber % 8;
	pGPIOHanlde->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
	pGPIOHanlde->pGPIOx->AFR[temp1] |= (pGPIOHanlde->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));

	}

}

/****************************************************************************************************
  * @function		-	GPIO_DeInit
  *
  * @brief			-	This Function disables or disables peripheral clock for the given GPIO port
  *
  * @param[in]    		-	pGPIOx : GPIOx
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

/****************************************************************************************************
  * @function		-	GPIO_ReadFromInputPin
  *
  * @brief			-	This Function disables or disables peripheral clock for the given GPIO port
  *
  * @param[in]    		-	pGPIOx : GPIOx
  *
  * @param[in]    		-	PinNumber
  *
  * @retval			-	0 or 1
  *
  * @Note			-	None
  */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IRD >> PinNumber) & 0x00000001);
	return value;
}

/****************************************************************************************************
  * @function		-	GPIO_ReadFromInputPort
  *
  * @brief			-	This Function disables or disables peripheral clock for the given GPIO port
  *
  * @param[in]    		-	pGPIOx : GPIOx
  *
  * @retval			-	0 or 1
  *
  * @Note			-	None
  */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IRD ;
	return value;
} 
/****************************************************************************************************
  * @function		-	GPIO_WriteToOutputPin
  *
  * @brief			-	This Function disables or disables peripheral clock for the given GPIO port
  *
  * @param[in]    		-	pGPIOx : GPIOx
  *
  * @param[in]    		-	PinNumber
  *
  * @param[in]    		-	Value
  *
  * @retval			-	None
  *
  * @Note			-	None
*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		// Write 1 to the ouput data registers at the bit field corresponding to the pin
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		// Write 0 to the ouput data registers at the bit field corresponding to the pin
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/****************************************************************************************************
  * @function		-	GPIO_WriteToOutputPort
  *
  * @brief			-	This Function disables or disables peripheral clock for the given GPIO port
  *
  * @param[in]    		-	pGPIOx : GPIOx
  *
  *
  * @param[in]    		-	Value
  *
  * @retval			-	None
  *
  * @Note			-	None
*/

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/****************************************************************************************************
  * @function		-	GPIO_ToggleOutputPin
  *
  * @brief			-	This Function disables or disables peripheral clock for the given GPIO port
  *
  * @param[in]    		-	pGPIOx : GPIOx
  *
  * @param[in]    		-	PinNumber
  *
  *
  * @retval			-	None
  *
  * @Note			-	None
*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/****************************************************************************************************
  * @function		-	GPIO_IRQInterruptConfig
  *
  * @brief			-	This Function IRQ Configuration and ISR handling
  *
  * @param[in]    	-	IRQNumber
  *
  *
  * @param[in] 		- 	ENorDIS
  *
  * @retval			-	None
  *
  * @Note			-	None
*/

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDIS)
{
	if(ENorDIS == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			// Program IRER0 Register
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			// Program IRER1 Register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}else if(IRQNumber >=64 && IRQNumber < 96)
		{
			// Program IRER2 Register
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}

	}else
	{
		if(IRQNumber <= 31)
		{
			// Program IRER0 Register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			// Program IRER1 Register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}else if(IRQNumber >=64 && IRQNumber < 96)
		{
			// Program IRER2 Register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
}

/****************************************************************************************************
  * @function		-	GPIO_IRQPriorityConfig
  *
  * @brief			-	This Function
  *
  * @param[in]    	-	IRQNumber
  *
  * @param[in]    	-	IRQPriority
  *
  * @retval			-	None
  *
  * @Note			-	None
*/

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. Fisrt lets find out the ipr register
	//uint8_t	iprx = IRQNumber / 4;

	uint8_t	iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*NVIC_PR_BASEADDR |= (IRQPriority << shift_amount);

}
/****************************************************************************************************
  * @function		-	GPIO_IRQHandling
  *
  * @brief			-	This Function
  *
  * @param[in]    	-	PinNumber
  *
  * @retval			-	None
  *
  * @Note			-	None
*/

void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear the exti pr register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		//clear
		EXTI->PR |= ( 1 << PinNumber);
	}
}
