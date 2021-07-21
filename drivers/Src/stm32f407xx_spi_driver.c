/*
 * stm32f407xx_SPI_driver.c
 *
 *  Created on: Jul 15, 2021
 *      Author: Quy Long
 */

#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"


static void SPI_RXNE_InterruptHanlde(SPI_Handle_t *pSPIHanlde);

static void SPI_TXE_InterruptHanlde(SPI_Handle_t *pSPIHanlde);

static void SPI_OVR_ERR_InterruptHanlde(SPI_Handle_t *pSPIHanlde);
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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

	while(Len > 0)
	{
		// 1. Wait until RXEN bit is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		// 2. Check the DFF bit in CR1
		if( (pSPIx->CR1 & (1 << SPI_CR1_DFF)) )
		{
			// 16 bits DFF
			// Load the data from DR to RxBuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else
		{
			// 8 bits DFF
			pSPIx->DR = *pRxBuffer*Len--;
			pRxBuffer++;
		}
	}


}

/****************************************************************************************************
  * @function		-	SPI_SendDataIT
  *
  * @brief			-	This API to send data with interrupt mode
  *
  * @param[in]    	-	pSPIHanlde :
  *
  *	@param[in]    	-	pTxBuffer :
  *
  *	@param[in]    	-	Len :
  *
  * @retval			-	None
  *
  * @Note			-	None
  */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHanlde, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHanlde->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		// 1. Save the TxBuffer address and Length information in some global variables
		pSPIHanlde->pTxBuffer = pTxBuffer;
		pSPIHanlde->TxLen = Len;

		// 2. Mark the SPI states as Busy in transmission so that no other code can take over same SPI peripheral
		//	until transmission is over
		pSPIHanlde->TxState = SPI_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit (Tx buffer empty interrupt enable )to get interrupt whenever TXE flag is set in SR
		pSPIHanlde->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;

}

/****************************************************************************************************
  * @function		-	SPI_ReceiveDataIT
  *
  * @brief			-	This
  *
  * @param[in]    	-	pSPIHanlde :
  *
  *	@param[in]    	-	pTxBuffer :
  *
  *	@param[in]    	-	Len :
  *
  * @retval			-	None
  *
  * @Note			-	None
  */

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHanlde, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHanlde->RxState;
		if(state != SPI_BUSY_IN_RX)
		{
			// 1. Save the RxBuffer address and Length information in some global variables
			pSPIHanlde->pRxBuffer = pRxBuffer;
			pSPIHanlde->RxLen = Len;

			// 2. Mark the SPI states as Busy in transmission so that no other code can take over same SPI peripheral
			//	until transmission is over
			pSPIHanlde->RxState = SPI_BUSY_IN_RX;

			// 3. Enable the TXEIE control bit (Tx buffer empty interrupt enable )to get interrupt whenever TXE flag is set in SR
			pSPIHanlde->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
		}
		return state;
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

/*									Enter ISR
 * 										|
 * 							Understand which event caused
 * 							interrupt to trigger (check SR)
 * 			|							|						|
 * 	Interrupt is due to			Interrupt is due to		Interrupt is due to
 * 	setting of RXNE flag			setting of TXE flag		setting of ERROR flag
 * 			|							|						|
 *	 Handle RXNE event			Handle TXE event			Handle Error
 *
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t	temp1, temp2;
	// First lets check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		// Handle TXE
		SPI_TXE_InterruptHanlde(pHandle);
	}

	// lets check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		// Handle RXNE
		SPI_RXNE_InterruptHanlde(pHandle);

	}

	// Check for OVR flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if(temp1 && temp2)
	{
		// Handle OVR error
		SPI_OVR_ERR_InterruptHanlde(pHandle);
	}

}

/*
 * Some functions for implementation
 */
static void SPI_TXE_InterruptHanlde(SPI_Handle_t *pSPIHanlde)
{
	// Check the DFF bit in CR1
	if( (pSPIHanlde->pSPIx->CR1 & (1 << SPI_CR1_DFF)) )
	{
		// 16 bits DFF
		pSPIHanlde->pSPIx->DR = *((uint16_t*)pSPIHanlde->pTxBuffer);
		pSPIHanlde->TxLen--;
		pSPIHanlde->TxLen--;
		(uint16_t*)pSPIHanlde->pTxBuffer++;
	}else
	{
		// 8 bits DFF
		pSPIHanlde->pSPIx->DR = *pSPIHanlde->pTxBuffer;
		pSPIHanlde->TxLen--;
		pSPIHanlde->pTxBuffer++;
	}

	if(!(pSPIHanlde->TxLen))
	{
		// TxLen is zero, so close the SPI transmission and inform the Application that TX is over

		// This prevents interrupts from setting of TXE flag
		SPI_CloseTransmission(pSPIHanlde);

		// Inform back to Application
		SPI_AppEventCallback(pSPIHanlde, SPI_EVENT_TX_CMPLT);

	}
}

static void SPI_RXNE_InterruptHanlde(SPI_Handle_t *pSPIHanlde)
{
	// Check the DFF bit in CR1
		if( (pSPIHanlde->pSPIx->CR1 & (1 << SPI_CR1_DFF)) )
		{
			// 16 bits DFF
			*((uint16_t*)pSPIHanlde->pRxBuffer) = (uint16_t)(pSPIHanlde->pSPIx->DR);
			pSPIHanlde->RxLen--;
			pSPIHanlde->RxLen--;
			pSPIHanlde->pRxBuffer--;
			pSPIHanlde->pRxBuffer--;
		}else
		{
			// 8 bits DFF
			*(pSPIHanlde->pRxBuffer) = (uint8_t)(pSPIHanlde->pSPIx->DR) ;
			pSPIHanlde->RxLen--;
			pSPIHanlde->pRxBuffer--;
		}

		if(!(pSPIHanlde->RxLen))
		{
			// RxLen is zero, turn off the Rx Interrupt
			SPI_CloseReception(pSPIHanlde);

			// Inform back to Application
			SPI_AppEventCallback(pSPIHanlde, SPI_EVENT_RX_CMPLT);

		}

}

static void SPI_OVR_ERR_InterruptHanlde(SPI_Handle_t *pSPIHanlde)
{
	uint8_t temp;

	// Clear the OVER Flag
	if(pSPIHanlde->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHanlde->pSPIx->DR;
		temp = pSPIHanlde->pSPIx->SR;
	}
	(void)temp;
	// Inform App
	SPI_AppEventCallback(pSPIHanlde, SPI_EVENT_OVR_ERR);
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
  * @function		-	SPI_CloseTransmission
  *
  * @brief			-	This Function
  *
  * @param[in]    	-	pSPIHandle
  *
  * @retval			-	None
  *
  * @Note			-	None
*/

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

/****************************************************************************************************
  * @function		-	SPI_CloseReception
  *
  * @brief			-	This Function
  *
  * @param[in]    	-	pSPIHandle
  *
  * @retval			-	None
  *
  * @Note			-	None
*/
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}
/****************************************************************************************************
  * @function		-	SPI_ClearOVRFlag
  *
  * @brief			-	This Function
  *
  * @param[in]    	-	pSPIHandle
  *
  * @retval			-	None
  *
  * @Note			-	None
*/
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
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

/****************************************************************************************************
  * @function		-	SPI_SSOEConfig
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
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDIS)
{
	if(ENorDIS == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

__weak void SPI_AppEventCallback(SPI_Handle_t *pSPIHanlde, uint8_t AppEvent)
{
	// This function is a weak implementation. The Application may override this function

}
