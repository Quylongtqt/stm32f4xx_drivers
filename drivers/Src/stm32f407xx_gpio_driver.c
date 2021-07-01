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
  * @Function		-	GIPO_PeriperalClockControl
  *
  * @brief			-	This Function enables or disables peripheral clock for the given GPIO port
  *
  * @param    		-	pGPIOx : Based address of the GPIO peripheral
  * @param  		-	ENorDIS: ENABLE or DISABLE
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
void GIPO_PeriperalClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDIS)

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
  * @Function		-	GIPO_Init
  *
  * @brief			-	This Function
  *
  * @param    		-	pGPIOHanlde : GPIOx
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
void GIPO_Init(GPIO_Handle_t *pGPIOHanlde)
{
	uint32_t temp = 0;	//temp. register
	/* 1. Configure the mode of gpio pin */
	if(pGPIOHanlde->GIPO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// the non interrupt mode
		temp = (pGPIOHanlde->GIPO_PinConfig.GPIO_PinMode << (2 * pGPIOHanlde->GIPO_PinConfig.GPIO_PinNumber));
		pGPIOHanlde->pGPIOx->MODER &= ~( 0x3 << pGPIOHanlde->GIPO_PinConfig.GPIO_PinNumber); // Clearing
		pGPIOHanlde->pGPIOx->MODER |= temp;
	}else
	{
		// the interrupt mode
	}
	temp = 0;

	/* 2. Configure the speed */
	temp = (pGPIOHanlde->GIPO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHanlde->GIPO_PinConfig.GPIO_PinNumber));
	pGPIOHanlde->pGPIOx->OSPEEDER &= ~( 0x3 << pGPIOHanlde->GIPO_PinConfig.GPIO_PinNumber); // Clearing
	pGPIOHanlde->pGPIOx->OSPEEDER |= temp;

	temp = 0;
	/* 3. Configure the PuPd settings */
	temp = (pGPIOHanlde->GIPO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHanlde->GIPO_PinConfig.GPIO_PinNumber));
	pGPIOHanlde->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHanlde->GIPO_PinConfig.GPIO_PinNumber); // Clearing
	pGPIOHanlde->pGPIOx->PUPDR |= temp;

	temp = 0;
	/* 4. Configure the OpType */
	temp = (pGPIOHanlde->GIPO_PinConfig.GPIO_PinOPType << (pGPIOHanlde->GIPO_PinConfig.GPIO_PinNumber));
	pGPIOHanlde->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHanlde->GIPO_PinConfig.GPIO_PinNumber); // Clearing
	pGPIOHanlde->pGPIOx->OTYPER |= temp;

	/* 5. Configure the Alt Functionality */
	if(pGPIOHanlde->GIPO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// Configure the alt function registers
	uint8_t temp1, temp2;

	temp1 = pGPIOHanlde->GIPO_PinConfig.GPIO_PinNumber / 8;
	temp2 = pGPIOHanlde->GIPO_PinConfig.GPIO_PinNumber % 8;
	pGPIOHanlde->pGPIOx->AFR[temp1] &= (0xF << 4 * temp2);
	pGPIOHanlde->pGPIOx->AFR[temp1] |= (pGPIOHanlde->GIPO_PinConfig.GPIO_PinAltFunMode << 4 * temp2);

	}

}

/****************************************************************************************************
  * @Function		-	GIPO_DeInit
  *
  * @brief			-	This Function disables or disables peripheral clock for the given GPIO port
  *
  * @param    		-	pGPIOx : GPIOx
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
void GIPO_DeInit(GPIO_RegDef_t *pGPIOx)
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
  * @Function		-	GIPO_ReadFromInputPin
  *
  * @brief			-	This Function disables or disables peripheral clock for the given GPIO port
  *
  * @param    		-	pGPIOx : GPIOx
  *
  * @param    		-	PinNumber
  *
  * @retval			-	0 or 1
  *
  * @Note			-	None
  */
uint8_t GIPO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IRD >> PinNumber) & 0x00000001);
	return value;
}

/****************************************************************************************************
  * @Function		-	GIPO_ReadFromInputPort
  *
  * @brief			-	This Function disables or disables peripheral clock for the given GPIO port
  *
  * @param    		-	pGPIOx : GPIOx
  *
  * @retval			-	0 or 1
  *
  * @Note			-	None
  */

uint16_t GIPO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IRD ;
	return value;
} 
/****************************************************************************************************
  * @Function		-	GIPO_WriteToOutputPin
  *
  * @brief			-	This Function disables or disables peripheral clock for the given GPIO port
  *
  * @param    		-	pGPIOx : GPIOx
  *
  * @param    		-	PinNumber
  *
  * @param    		-	Value 
  *
  * @retval			-	None
  *
  * @Note			-	None
*/
void GIPO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
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
  * @Function		-	GIPO_WriteToOutputPort
  *
  * @brief			-	This Function disables or disables peripheral clock for the given GPIO port
  *
  * @param    		-	pGPIOx : GPIOx
  *
  *
  * @param    		-	Value
  *
  * @retval			-	None
  *
  * @Note			-	None
*/

void GIPO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/****************************************************************************************************
  * @Function		-	GIPO_ToggleOutputPin
  *
  * @brief			-	This Function disables or disables peripheral clock for the given GPIO port
  *
  * @param    		-	pGPIOx : GPIOx
  *
  * @param    		-	PinNumber
  *
  *
  * @retval			-	None
  *
  * @Note			-	None
*/
void GIPO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR handling
 */
void GIPO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDIS)
{

}
void GIPO_IRQHandling(uint8_t PinNumber)
{

}
