/*
 * stm32f407xx_SPI_driver.c
 *
 *  Created on: Jul 15, 2021
 *      Author: Quy Long
 */

#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
/****************************************************************************************************
 * 								APIs supported by this driver
 *			 For more information about APIs check the function definitions
 *
 ****************************************************************************************************/

/****************************************************************************************************
  * @function		-	SPI_PeriperalClockControl
  *
  * @brief			-	This Function enables or disables peripheral clock for the given spi port
  *
  * @param[in]    		-	pSPIx : Based address of the spi peripheral
  * @param[in]  		-	ENorDIS: ENABLE or DISABLE
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
/*
 *
 */void SPI_PeriperalClockControl(SPI_RegDef_t *pSPIx, uint8_t ENorDIS)
{
	if(ENorDIS == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}else if(pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}
			else if(pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}

		}else
		{
			//Todo
		}
}

/****************************************************************************************************
  * @function		-	SPI_Init
  *
  * @brief			-	This Function
  *
  * @param[in]    	-	pSPIHanlde : SPIx
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
void SPI_Init(SPI_Handle_t *pSPIHanlde)
{

	// Enable the peripheral clock
	SPI_PeriperalClockControl(pSPIHanlde->pSPIx, ENABLE);

	// First lets configure the SPI_CR1 register
	uint32_t	tempreg = 0;

	// 1. Configure the device mode
	tempreg |= pSPIHanlde->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. Configure the bus config
	if(pSPIHanlde->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHanlde->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// BIDI mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHanlde->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// BIDI mode should be clear and RxOnly bit should be set
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the SPI serial clock speed (baud rate)
	tempreg |= pSPIHanlde->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. Configure the DFF
	tempreg |= pSPIHanlde->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// 5. Configure the CPOL
	tempreg |= pSPIHanlde->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// 6. Configure the CPHA
	tempreg |= pSPIHanlde->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	// 7. Configure the CPHA
	tempreg |= pSPIHanlde->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	// Lets set configure for SPI_CR1 register
	pSPIHanlde->pSPIx->CR1 = tempreg;


}

/****************************************************************************************************
  * @function		-	SPI_DeInit
  *
  * @brief			-	This Function disables or disables peripheral clock for the given spi port
  *
  * @param[in]    	-	pSPIx :
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}

/****************************************************************************************************
  * @function		-	SPI_GetFlagStatus
  *
  * @brief			-	This
  *
  * @param[in]    	-	pSPIx :
  *
  * @param[in]    	-	pTxBuffer :
  *
  * @param[in]    	-	Len :
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/****************************************************************************************************
  * @function		-	SPI_SendData
  *
  * @brief			-	This
  *
  * @param[in]    	-	pSPIx :
  *
  * @param[in]    	-	pTxBuffer :
  *
  * @param[in]    	-	Len :
  *
  * @retval			-	None
  *
  * @Note			-	This is Blocking call
  */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		// 1. Wait until TXE bit is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2. Check the DFF bit in CR1
		if( (pSPIx->CR1 & (1 << SPI_CR1_DFF)) )
		{
			// 16 bits DFF
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			// 8 bits DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}

}
/****************************************************************************************************
  * @function		-	SPI_ReceiveData
  *
  * @brief			-	This
  *
  * @param[in]    	-	pSPIx :
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx)
{

}
/****************************************************************************************************
  * @function		-	SPI_IRQInterruptConfig
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

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDIS)
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
  * @function		-	SPI_IRQPriorityConfig
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. Fisrt lets find out the ipr register
	//uint8_t	iprx = IRQNumber / 4;

	uint8_t	iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*NVIC_PR_BASEADDR |= (IRQPriority << shift_amount);

}
/****************************************************************************************************
  * @function		-	SPI_IRQHandling
  *
  * @brief			-	This Function
  *
  * @param[in]    	-	PinNumber
  *
  * @retval			-	None
  *
  * @Note			-	None
*/

void SPI_IRQHandling(SPI_Handle_t *pSPIHanlde)
{
	// Clear the exti pr register corresponding to the pin number

}

/****************************************************************************************************
  * @function		-	SPI_PeripheralControl
  *
  * @brief			-	This Function
  *
  * @param[in]    	-	pSPIx
  *
  * @param[in]    	-	ENorDIS
  *
  * @retval			-	None
  *
  * @Note			-	None
*/

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDIS)
{
	if(ENorDIS == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}

}

/****************************************************************************************************
  * @function		-	SPI_SSIConfig
  *
  * @brief			-	This Function
  *
  * @param[in]    	-	pSPIx
  *
  * @param[in]    	-	ENorDIS
  *
  * @retval			-	None
  *
  * @Note			-	None
*/

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDIS)
{
	if(ENorDIS == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}

}
