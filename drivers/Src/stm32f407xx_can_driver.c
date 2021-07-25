/*
 * stm32f407xx_can_driver.c
 *
 *  Created on: Jul 24, 2021
 *      Author: Quy Long
 */
#include "stm32f407xx.h"
#include "stm32f407xx_can_driver.h"

#define assert_param(expr) ((void)0)

/* CAN Master Control Register bits */
#define MCR_DBF           ((uint32_t)0x00010000) /* software master reset */

/* CAN Mailbox Transmit Request */
#define TMIDxR_TXRQ       ((uint32_t)0x00000001) /* Transmit mailbox request */

/* CAN Filter Master Register bits */
#define FMR_FINIT         ((uint32_t)0x00000001) /* Filter init mode */

/* Time out for INAK bit */
#define INAK_TIMEOUT      ((uint32_t)0x0000FFFF)
/* Time out for SLAK bit */
#define SLAK_TIMEOUT      ((uint32_t)0x0000FFFF)

/* Flags in TSR register */
#define CAN_FLAGS_TSR     ((uint32_t)0x08000000)
/* Flags in RF1R register */
#define CAN_FLAGS_RF1R    ((uint32_t)0x04000000)
/* Flags in RF0R register */
#define CAN_FLAGS_RF0R    ((uint32_t)0x02000000)
/* Flags in MSR register */
#define CAN_FLAGS_MSR     ((uint32_t)0x01000000)
/* Flags in ESR register */
#define CAN_FLAGS_ESR     ((uint32_t)0x00F00000)

/* Mailboxes definition */
#define CAN_TXMAILBOX_0   ((uint8_t)0x00)
#define CAN_TXMAILBOX_1   ((uint8_t)0x01)
#define CAN_TXMAILBOX_2   ((uint8_t)0x02)

#define CAN_MODE_MASK     ((uint32_t) 0x00000003)

/****************************************************************************************************
  * @function		-	CAN_PeriperalClockControl
  *
  * @brief			-	This Function enables or disables peripheral clock for the given spi port
  *
  * @param[in]    		-	pCANx : Based address of the spi peripheral
  * @param[in]  		-	ENorDIS: ENABLE or DISABLE
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
/*
 *
 */
void CAN_PeriperalClockControl(CAN_RegDef_t *pCANx, uint8_t ENorDIS)
{
	if(ENorDIS == ENABLE)
		{
			if(pCANx == CAN1)
			{
				CAN1_PCLK_EN();
			}else if(pCANx == CAN2)
			{
				CAN2_PCLK_EN();
			}

		}else
		{
			//Todo
		}
}

/****************************************************************************************************
  * @function		-	CAN_Init
  *
  * @brief			-	This Function
  *
  * @param[in]    	-	pCANHanlde
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
uint8_t CAN_Init(CAN_Handle_t *pCANHanlde)
{
	uint8_t InitStatus = CAN_InitStatus_Failed;
	uint32_t wait_ack = 0;

	// Enable the peripheral clock
	CAN_PeriperalClockControl(pCANHanlde->pCANx, ENABLE);

	/* Check parameters */
	assert_param(IS_CAN_ALL_PERIPH(pCANHanlde->pCANx));
	assert_param(IS_FUNCTIONAL_STATE(pCANHanlde->CANConfig->CAN_TTCM));
	assert_param(IS_FUNCTIONAL_STATE(pCANHanlde->CANConfig->CAN_ABOM));
	assert_param(IS_FUNCTIONAL_STATE(pCANHanlde->CANConfig->CAN_AWUM));
	assert_param(IS_FUNCTIONAL_STATE(pCANHanlde->CANConfig->CAN_NART));
	assert_param(IS_FUNCTIONAL_STATE(pCANHanlde->CANConfig->CAN_RFLM));
	assert_param(IS_FUNCTIONAL_STATE(pCANHanlde->CANConfig->CAN_TXFP));
	assert_param(IS_CAN_MODE(pCANHanlde->CANConfig->CAN_Mode));
	assert_param(IS_CAN_SJW(pCANHanlde->CANConfig->CAN_SJW));
	assert_param(IS_CAN_BS1(pCANHanlde->CANConfig->CAN_BS1));
	assert_param(IS_CAN_BS2(pCANHanlde->CANConfig->CAN_BS2));
	assert_param(IS_CAN_PRESCALER(pCANHanlde->CANConfig->CAN_Prescaler));

	/* Exit from sleep mode */
	pCANHanlde->pCANx->MCR &= ~(1 << CAN_MCR_SLEEP);

	/* Request initialization */
	pCANHanlde->pCANx->MCR |= CAN_MCR_INRQ;

	/* Wait the acknowledge */
	while ( ((pCANHanlde->pCANx->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) && (wait_ack != INAK_TIMEOUT) )
	{
	wait_ack++;
	}

	/* Check acknowledge */
	if((pCANHanlde->pCANx->MSR & CAN_MSR_INAK) != CAN_MSR_INAK)
	{
		InitStatus = CAN_InitStatus_Failed;
	}else
	{
		/* Set the time triggered communication mode */
		if (pCANHanlde->CANConfig.CAN_TTCM == ENABLE)
		{
			pCANHanlde->pCANx->MSR |= CAN_MCR_TTCM;
		}else
		{
			pCANHanlde->pCANx->MSR &= ~(uint32_t)CAN_MCR_TTCM;
		}
		/* Set the automatic bus-off management */
		if (pCANHanlde->CANConfig.CAN_ABOM == ENABLE)
		{
			pCANHanlde->pCANx->MSR |= CAN_MCR_ABOM;
		}else
		{
			pCANHanlde->pCANx->MSR &= ~(uint32_t)CAN_MCR_ABOM;
		}
		/* Set the automatic wake-up mode */
		if (pCANHanlde->CANConfig.CAN_AWUM == ENABLE)
		{
			pCANHanlde->pCANx->MSR |= CAN_MCR_AWUM;
		}else
		{
			pCANHanlde->pCANx->MSR &= ~(uint32_t)CAN_MCR_AWUM;
		}
		/* Set the no automatic retransmission */
		if (pCANHanlde->CANConfig.CAN_NART == ENABLE)
		{
			pCANHanlde->pCANx->MSR |= CAN_MCR_NART;
		}else
		{
			pCANHanlde->pCANx->MSR &= ~(uint32_t)CAN_MCR_NART;
		}
		/* Set the receive FIFO locked mode */
		if (pCANHanlde->CANConfig.CAN_RFLM == ENABLE)
		{
			pCANHanlde->pCANx->MSR |= CAN_MCR_RFLM;
		}else
		{
			pCANHanlde->pCANx->MSR &= ~(uint32_t)CAN_MCR_RFLM;
		}
		/* Set the transmit FIFO priority */
		if (pCANHanlde->CANConfig.CAN_TXFP == ENABLE)
		{
			pCANHanlde->pCANx->MSR |= CAN_MCR_TXFP;
		}else
		{
			pCANHanlde->pCANx->MSR &= ~(uint32_t)CAN_MCR_TXFP;
		}
		/* Set the bit timing register */
		pCANHanlde->pCANx->BTR = ((uint32_t)pCANHanlde->CANConfig.CAN_Mode << 30) | \
								 ((uint32_t)pCANHanlde->CANConfig.CAN_SJW  << 24) | \
								 ((uint32_t)pCANHanlde->CANConfig.CAN_BS1  << 16) | \
								 ((uint32_t)pCANHanlde->CANConfig.CAN_BS2  << 20) | \
								 ((uint32_t)pCANHanlde->CANConfig.CAN_Prescaler - 1);
		/* Request leave initialization */
		pCANHanlde->pCANx->MCR &= ~(uint32_t)CAN_MCR_INRQ;

		/* Wait the acknowledge */
		wait_ack = 0;
		while((pCANHanlde->pCANx->MSR & CAN_MSR_INAK) && (wait_ack != INAK_TIMEOUT))
		{
			wait_ack++;
		}
		/* ...and check acknowledged */
		if((pCANHanlde->pCANx->MSR & CAN_MSR_INAK) == CAN_MSR_INAK)
		{
			InitStatus = CAN_InitStatus_Failed;
		}else
		{
			InitStatus = CAN_InitStatus_Success;
		}
	}
	/* At this step, return the status of initialization */
	return InitStatus;
}

/****************************************************************************************************
  * @function		-	CAN_DeInit
  *
  * @brief			-	This Function disables or disables peripheral clock for the given spi port
  *
  * @param[in]    	-	CANx : CAN1 or CAN2
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
void CAN_DeInit (CAN_RegDef_t *CANx)
{

	/* Check the parameters */
	  assert_param(IS_CAN_ALL_PERIPH(CANx));

	if(CANx == CAN1)
	{
		CAN1_REG_RESET();
	}else
	{
		CAN2_REG_RESET();
	}

}


