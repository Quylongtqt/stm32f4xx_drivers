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
/*
===============================================================================
              ##### Initialization and Configuration functions #####
 ===============================================================================
    [..] This section provides functions allowing to
      (+) Initialize the CAN peripherals : Prescaler, operating mode, the maximum
          number of time quanta to perform resynchronization, the number of time
          quanta in Bit Segment 1 and 2 and many other modes.
          Refer to  @ref CAN_InitTypeDef  for more details.
      (+) Configures the CAN reception filter.
      (+) Select the start bank filter for slave CAN.
      (+) Enables or disables the Debug Freeze mode for CAN
      (+)Enables or disables the CAN Time Trigger Operation communication mode
*/

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
  * @brief			-	This Function initializes the CAN peripheral according to the specified
  *         			parameters in the CAN_Handle_t.
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
  * @brief			-	This Function Deinitializes the CAN peripheral registers to their default reset values.
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

/****************************************************************************************************
  * @function		-	CAN_FilterInit
  *
  * @brief			-	This Function Configures the CAN reception filter according to the specified
  *         			parameters in the CAN_FilterInitStruct
  *
  * @param[in]    	-	CAN_FilterInitStruct
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
void CAN_FilterInit(CAN_FilterInitTypeDef* CAN_FilterInitStruct)
{
	uint32_t filter_number_bit_pos = 0;

	/* Check parameter */
	assert_param(IS_CAN_FILTER_NUMBER(CAN_FilterInitStruct->CAN_FilterNumber));
	assert_param(IS_CAN_FILTER_NUMBER(CAN_FilterInitStruct->CAN_FilterNumber));
	assert_param(IS_CAN_FILTER_MODE(CAN_FilterInitStruct->CAN_FilterMode));
	assert_param(IS_CAN_FILTER_SCALE(CAN_FilterInitStruct->CAN_FilterScale));
	assert_param(IS_CAN_FILTER_FIFO(CAN_FilterInitStruct->CAN_FilterFIFOAssignment));
	assert_param(IS_FUNCTIONAL_STATE(CAN_FilterInitStruct->CAN_FilterActivation));


	filter_number_bit_pos = ((uint32_t) 1) << CAN_FilterInitStruct->CAN_FilterNumber;

	/* Initialization mode for the filter */
	/* These registers can be written only when the filter initialization mode is set (FINIT=1) in the
	CAN_FMR register.*/
	CAN1->FMR |= FMR_FINIT;

	/*Filter Deactivation */
	CAN1->FA1R &= ~(uint32_t)filter_number_bit_pos;

	/* Filter Scale */
	if(CAN_FilterInitStruct->CAN_FilterScale == CAN_FilterScale_16bit)
	{
		/* 16-bit scale for the filter */
		CAN1->FM1R &= ~(uint32_t)filter_number_bit_pos;
		S
		/* First 16-bit identifier and First 16-bit mask */
		/* Or First 16-bit identifier and Second 16-bit identifier */
		CAN1->sFilterRegister[CAN_FilterInitStruct->CAN_FilterNumber].FR1 =
			((0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterMaskIdLow) << 16) |
			 (0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterIdLow);

		/* Second 16-bit identifier and Second 16-bit mask */
		/* Or Third 16-bit identifier and Fourth 16-bit identifier */
		CAN1->sFilterRegister[CAN_FilterInitStruct->CAN_FilterNumber].FR2 =
			((0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterMaskIdHigh) << 16) |
			 (0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterIdHigh);
	}

	if(CAN_FilterInitStruct->CAN_FilterScale == CAN_FilterScale_32bit)
	{
		/* 32-bit scale for the filter */
		CAN1->FS1R |= filter_number_bit_pos;

		/* 32-bit identifier or First 32-bit identifier */
		CAN1->sFilterRegister[CAN_FilterInitStruct->CAN_FilterNumber].FR1 =
			((0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterIdHigh) << 16) |
			(0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterIdLow);

		/* 32-bit mask or Second 32-bit identifier */
		CAN1->sFilterRegister[CAN_FilterInitStruct->CAN_FilterNumber].FR2 =
			((0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterMaskIdHigh) << 16) |
			(0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterMaskIdLow);
	}

	/* Filter Mode */
	if (CAN_FilterInitStruct->CAN_FilterMode == CAN_FilterMode_IdMask)
	{
		/*Id/Mask mode for the filter*/
		CAN1->FM1R &= ~(uint32_t)filter_number_bit_pos;
	}else /* CAN_FilterInitStruct->CAN_FilterMode == CAN_FilterMode_IdList */
	{
		/*Identifier list mode for the filter*/
		CAN1->FM1R |= (uint32_t)filter_number_bit_pos;
	}

	/* Filter FIFO assignment */
	if(CAN_FilterInitStruct->CAN_FilterFIFOAssignment == CAN_Filter_FIFO0)
	{
		/* FIFO 0 assignation for the filter */
		CAN1->FFA1R &= ~(uint32_t)filter_number_bit_pos;
	}

	if(CAN_FilterInitStruct->CAN_FilterFIFOAssignment == CAN_Filter_FIFO1)
	{
		/* FIFO 0 assignation for the filter */
		CAN1->FFA1R |= (uint32_t)filter_number_bit_pos;
	}

	/* Filter activation */
	if(CAN_FilterInitStruct->CAN_FilterActivation == ENABLE)
	{
		CAN1->FA1R |= filter_number_bit_pos;
	}

	/* Leave the initialization mode for the filter */
	CAN1->FMR &= ~FMR_FINIT;

}
/****************************************************************************************************
  * @function		-	CAN_StructInit
  *
  * @brief			-	This Function Reset CAN init structure parameters values
  *
  * @param[in]    	-	CAN_InitStruct
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
void CAN_StructInit(CAN_Config_t* CAN_InitStruct)
{
  /* Reset CAN init structure parameters values */

  /* Initialize the time triggered communication mode */
  CAN_InitStruct->CAN_TTCM = DISABLE;

  /* Initialize the automatic bus-off management */
  CAN_InitStruct->CAN_ABOM = DISABLE;

  /* Initialize the automatic wake-up mode */
  CAN_InitStruct->CAN_AWUM = DISABLE;

  /* Initialize the no automatic retransmission */
  CAN_InitStruct->CAN_NART = DISABLE;

  /* Initialize the receive FIFO locked mode */
  CAN_InitStruct->CAN_RFLM = DISABLE;

  /* Initialize the transmit FIFO priority */
  CAN_InitStruct->CAN_TXFP = DISABLE;

  /* Initialize the CAN_Mode member */
  CAN_InitStruct->CAN_Mode = CAN_Mode_Normal;

  /* Initialize the CAN_SJW member */
  CAN_InitStruct->CAN_SJW = CAN_SJW_1tq;

  /* Initialize the CAN_BS1 member */
  CAN_InitStruct->CAN_BS1 = CAN_BS1_4tq;

  /* Initialize the CAN_BS2 member */
  CAN_InitStruct->CAN_BS2 = CAN_BS2_3tq;

  /* Initialize the CAN_Prescaler member */
  CAN_InitStruct->CAN_Prescaler = 1;
}

/****************************************************************************************************
  * @function		-	CAN_SlaveStartBank
  *
  * @brief			-	This Function Select the start bank filter for slave CAN.
  *
  * @param[in]    	-	CAN_BankNumber: Select the start slave bank filter from 1..27.
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
void CAN_SlaveStartBank(uint8_t CAN_BankNumber)
{
	/* Check the parameters */
	assert_param(IS_CAN_BANKNUMBER(CAN_BankNumber));

	/* Enter Initialization mode for the filter */
	CAN1->FMR |= FMR_FINIT;

	/* Select the start slave bank */
	CAN1->FMR &= (uint32_t) 0xFFFFC0F1;
	CAN1->FMR |= (uint32_t) (CAN_BankNumber) << 8;

	/* Leave the initialization mode for the filter */
	CAN1->FMR &= ~FMR_FINIT;
}

/****************************************************************************************************
  * @function		-	CAN_DBGFreeze
  *
  * @brief			-	This Function Enables or disables the DBG Freeze for CAN.
  *
  * @param[in]    	-	CANx: where x can be 1 or 2 to to select the CAN peripheral.
  *
  * @param[in]    	-	NewState: new state of the CAN peripheral.
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
void CAN_DBGFreeze(CAN_RegDef_t* CANx, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));

	if(NewState != DISABLE)
	{
		/* Enable Debug Freeze */
		CANx->MCR |= MCR_DBF;
	}else
	{
		/* Enable Debug Freeze */
		CANx->MCR &= ~MCR_DBF;
	}
}

/****************************************************************************************************
  * @function		-	CAN_TTComModeCmd
  *
  * @brief			-	This Function  Enables or disables the CAN Time TriggerOperation communication mode.
  *
  * @param[in]    	-	CANx: where x can be 1 or 2 to to select the CAN peripheral.
  *
  * @param[in]    	-	NewState: new state of the CAN peripheral.
  *
  * @retval			-	None
  *
  * @Note			-	DLC must be programmed as 8 in order Time Stamp (2 bytes) to be
  *         			sent over the CAN bus.
  */
void CAN_TTComModeCmd(CAN_RegDef_t* CANx, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));

	if (NewState != DISABLE)
	{
	  /* Enable the TTCM mode */
	  CANx->MCR |= CAN_MCR_TTCM;

	  /* Set TGT bits */
	  /*!< CAN mailbox data length control and time stamp register */
	  CANx->sTxMailBox[0].TDTR |= ((uint32_t)CAN_TDT0R_TGT);
	  CANx->sTxMailBox[1].TDTR |= ((uint32_t)CAN_TDT1R_TGT);
	  CANx->sTxMailBox[2].TDTR |= ((uint32_t)CAN_TDT2R_TGT);
	}else
	{
	/* Disable the TTCM mode */
	CANx->MCR &= (uint32_t)(~(uint32_t)CAN_MCR_TTCM);

	/* Reset TGT bits */
	CANx->sTxMailBox[0].TDTR &= ((uint32_t)~CAN_TDT0R_TGT);
	CANx->sTxMailBox[1].TDTR &= ((uint32_t)~CAN_TDT1R_TGT);
	CANx->sTxMailBox[2].TDTR &= ((uint32_t)~CAN_TDT2R_TGT);
	}
}

/*
===============================================================================
                ##### CAN Frames Transmission functions #####
 ===============================================================================
    [..] This section provides functions allowing to
      (+) Initiate and transmit a CAN frame message (if there is an empty mailbox).
      (+) Check the transmission status of a CAN Frame
      (+) Cancel a transmit request
 */

/****************************************************************************************************
  * @function		-	CAN_TTComModeCmd
  *
  * @brief			-	This Function Initiates and transmits a CAN frame message.
  *
  * @param[in]    	-	CANx: where x can be 1 or 2 to to select the CAN peripheral.
  *
  * @param[in]    	-	TxMessage: pointer to a structure which contains CAN Id, CAN DLC and CAN data.
  *
  * @retval			-	The number of the mailbox that is used for transmission or
  *        				CAN_TxStatus_NoMailBox if there is no empty mailbox.
  *
  * @Note			-	None
  *
  */

uint8_t	CAN_Transmit(CAN_RegDef_t *CANx, CAN_TxMsg* TxMessage)
{
	uint8_t transmit_mailbox = 0;
	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	assert_param(IS_CAN_IDTYPE(TxMessage->IDE));
	assert_param(IS_CAN_RTR(TxMessage->RTR));
	assert_param(IS_CAN_DLC(TxMessage->DLC));

	/* Select one empty transmit mailbox */
	if((CANx->TSR & CAN_TSR_TME0) == CAN_TSR_TME0)
	{
		transmit_mailbox = 0;
	}else if((CANx->TSR & CAN_TSR_TME1) == CAN_TSR_TME1)
	{
		transmit_mailbox = 1;
	}else if((CANx->TSR & CAN_TSR_TME2) == CAN_TSR_TME2)
	{
		transmit_mailbox = 2;
	}else
	{
		transmit_mailbox = CAN_TxStatus_NoMailBox;
	}

	/* Check whether the transmit_mailbox is available */
	if(transmit_mailbox != CAN_TxStatus_NoMailBox)
	{
		/* Set up the Id */
		CANx->sTxMailBox[transmit_mailbox].TIR &= TMIDxR_TXRQ;
		if(TxMessage->IDE == CAN_ID_STD)
		{
			assert_param(IS_CAN_STDID(TxMessage->StdId));
			CANx->sTxMailBox[transmit_mailbox].TIR |= ((TxMessage->StdId <<21) | \
														TxMessage->RTR);
		}else
		{
			assert_param(IS_CAN_EXTID(TxMessage->StdId));
			CANx->sTxMailBox[transmit_mailbox].TIR |= ((TxMessage->StdId <<3) | \
														TxMessage->IDE | \
														TxMessage->RTR);
		}

		/* Set up the DLC */
		TxMessage->DLC &= (uint8_t) 0x0000000F;
		CANx->sTxMailBox[transmit_mailbox].TDTR &= (uint32_t) 0xFFFFFFF0;
		CANx->sTxMailBox[transmit_mailbox].TDTR |= TxMessage->DLC;

		/* Set up the data field */
		CANx->sTxMailBox[transmit_mailbox].TDLR = 	((uint32_t)TxMessage->Data[3] << 24) | \
													((uint32_t)TxMessage->Data[2] << 16) | \
													((uint32_t)TxMessage->Data[1] << 8)  | \
													((uint32_t)TxMessage->Data[0] << 0);

		CANx->sTxMailBox[transmit_mailbox].TDHR =	((uint32_t)TxMessage->Data[7] << 24) | \
													((uint32_t)TxMessage->Data[6] << 16) | \
													((uint32_t)TxMessage->Data[5] << 8)  | \
													((uint32_t)TxMessage->Data[4] << 0);

		/* Request transmission */
		CANx->sTxMailBox[transmit_mailbox].TIR |= TMIDxR_TXRQ;
	}
	return transmit_mailbox;
}

/****************************************************************************************************
  * @function		-	CAN_TransmitStatus
  *
  * @brief			-	This Function Checks the transmission status of a CAN Frame.
  *
  * @param[in]    	-	CANx: where x can be 1 or 2 to to select the CAN peripheral.
  *
  * @param[in]    	-	TransmitMailbox: the number of the mailbox that is used for transmission.
  *
  * @retval			-	CAN_TxStatus_Ok if the CAN driver transmits the message,
  *         			CAN_TxStatus_Failed in an other case.
  *
  * @Note			-	None
  */

uint8_t CAN_TransmitStatus(CAN_RegDef_t* CANx, uint8_t TransmitMailbox)
{
	uint32_t state = 0;
	/* Check parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	assert_param(IS_CAN_TRANSMITMAILBOX(TransmitMailbox));

	switch (TransmitMailbox)
	{
	case (CAN_TXMAILBOX_0):
	  state =   CANx->TSR &  (CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0);
	  break;
	case (CAN_TXMAILBOX_1):
	  state =   CANx->TSR &  (CAN_TSR_RQCP1 | CAN_TSR_TXOK1 | CAN_TSR_TME1);
	  break;
	case (CAN_TXMAILBOX_2):
	  state =   CANx->TSR &  (CAN_TSR_RQCP2 | CAN_TSR_TXOK2 | CAN_TSR_TME2);
	  break;
	default:
	  state = CAN_TxStatus_Failed;
	  break;
	}
	switch (state)
  {
	  /* transmit pending  */
	case (0x0): state = CAN_TxStatus_Pending;
	  break;
	  /* transmit failed  */
	 case (CAN_TSR_RQCP0 | CAN_TSR_TME0): state = CAN_TxStatus_Failed;
	  break;
	 case (CAN_TSR_RQCP1 | CAN_TSR_TME1): state = CAN_TxStatus_Failed;
	  break;
	 case (CAN_TSR_RQCP2 | CAN_TSR_TME2): state = CAN_TxStatus_Failed;
	  break;
	  /* transmit succeeded  */
	case (CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0):state = CAN_TxStatus_Ok;
	  break;
	case (CAN_TSR_RQCP1 | CAN_TSR_TXOK1 | CAN_TSR_TME1):state = CAN_TxStatus_Ok;
	  break;
	case (CAN_TSR_RQCP2 | CAN_TSR_TXOK2 | CAN_TSR_TME2):state = CAN_TxStatus_Ok;
	  break;
	default: state = CAN_TxStatus_Failed;
	  break;
  }
	return (uint8_t) state;
}

/****************************************************************************************************
  * @function		-	CAN_CancelTransmit
  *
  * @brief			-	This Function Cancels a transmit request.
  *
  * @param[in]    	-	CANx: where x can be 1 or 2 to to select the CAN peripheral.
  *
  * @param[in]    	-	Mailbox: Mailbox number.
  *
  * @retval			-	None
  *
  * @Note			-	None
  */


void CAN_CancelTransmit(CAN_RegDef_t* CANx, uint8_t Mailbox)
{
	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	assert_param(IS_CAN_TRANSMITMAILBOX(Mailbox));

	/* abort transmission */
	switch(Mailbox)
	{
	case(CAN_TXMAILBOX_0): CANx->TSR |= CAN_TSR_ABRQ0;
		break;
	case(CAN_TXMAILBOX_0): CANx->TSR |= CAN_TSR_ABRQ1;
		break;
	case(CAN_TXMAILBOX_0): CANx->TSR |= CAN_TSR_ABRQ2;
		break;
	default:
		break;
	}
}

/*
===============================================================================
                ##### CAN Frames Reception functions #####
 ===============================================================================
    [..] This section provides functions allowing to
      (+) Receive a correct CAN frame
      (+) Release a specified receive FIFO (2 FIFOs are available)
      (+) Return the number of the pending received CAN frames
*/

/****************************************************************************************************
  * @function		-	CAN_Receive
  *
  * @brief			-	This Function Receives a correct CAN frame.
  *
  * @param[in]    	-	CANx: where x can be 1 or 2 to to select the CAN peripheral.
  *
  * @param[in]    	-	FIFONumber: Receive FIFO number, CAN_FIFO0 or CAN_FIFO1.
  *
  * @param[in]    	-	RxMessage: pointer to a structure receive frame which contains CAN Id,
  *         			CAN DLC, CAN data and FMI number.
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
void CAN_Receive(CAN_RegDef_t* CANx, uint8_t FIFONumber, CanRxMsg* RxMessage)
{
	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	assert_param(IS_CAN_FIFO(FIFONumber));

	/* Get the Id */
	RxMessage->IDE = (uint8_t)0x4 & CANx->sFIFOMailBox[FIFONumber].RIR;
	if(RxMessage->IDE == CAN_ID_STD)
	{
		/* Standard CAN */
		RxMessage->StdId = (uint32_t)0x000007FF & (CANx->sFIFOMailBox[FIFONumber].RIR>21);
	}else
	{
		/* Extended CAN */
		RxMessage->StdId = (uint32_t)0x1FFFFFFF & (CANx->sFIFOMailBox[FIFONumber].RIR>3);
	}
	RxMessage->RTR = (uint8_t)0x02 & CANx->sFIFOMailBox[FIFONumber].RIR;

	/* Get the DLC */
	RxMessage->RTR = (uint8_t)0x0F & CANx->sFIFOMailBox[FIFONumber].RDTR;

	/* Get the FMI */
	RxMessage->FMI = (uint16_t)0xFF & (CANx->sFIFOMailBox[FIFONumber].RDTR >> 8);

	/* Get the data field */
	RxMessage->Data[0] = (uint8_t) 0xFF & (CANx->sFIFOMailBox[FIFONumber].RDLR >> 0);
	RxMessage->Data[1] = (uint8_t) 0xFF & (CANx->sFIFOMailBox[FIFONumber].RDLR >> 8);
	RxMessage->Data[2] = (uint8_t) 0xFF & (CANx->sFIFOMailBox[FIFONumber].RDLR >> 16);
	RxMessage->Data[3] = (uint8_t) 0xFF & (CANx->sFIFOMailBox[FIFONumber].RDLR >> 24);
	RxMessage->Data[4] = (uint8_t) 0xFF & (CANx->sFIFOMailBox[FIFONumber].RDHR >> 0);
	RxMessage->Data[5] = (uint8_t) 0xFF & (CANx->sFIFOMailBox[FIFONumber].RDHR >> 8);
	RxMessage->Data[6] = (uint8_t) 0xFF & (CANx->sFIFOMailBox[FIFONumber].RDHR >> 16);
	RxMessage->Data[7] = (uint8_t) 0xFF & (CANx->sFIFOMailBox[FIFONumber].RDHR >> 24);

	/* Release the FIFO */
	if(FIFONumber == CAN_FIFO0)
	{
		/* Release FIFO0 */
		CANx->RF0R |= CAN_RF0R_RFOM0;
	}else
	{
		/* Release FIFO1 */
		CANx->RF1R |= CAN_RF1R_RFOM1;
	}
}

/****************************************************************************************************
  * @function		-	CAN_FIFORelease
  *
  * @brief			-	This Function Releases the specified receive FIFO.
  *
  * @param[in]    	-	CANx: where x can be 1 or 2 to to select the CAN peripheral.
  *
  * @param[in]    	-	FIFONumber: Receive FIFO number, CAN_FIFO0 or CAN_FIFO1.
  *
  * @retval			-	None
  *
  * @Note			-	None
  */
void CAN_FIFORelease(CAN_RegDef_t* CANx, uint8_t FIFONumber)
{
	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	assert_param(IS_CAN_FIFO(FIFONumber));
	/* Release the FIFO */
	if(FIFONumber == CAN_FIFO0)
	{
		/* Release FIFO0 */
		CANx->RF0R |= CAN_RF0R_RFOM0;
	}else
	{
		/* Release FIFO1 */
		CANx->RF1R |= CAN_RF1R_RFOM1;
	}
}

/****************************************************************************************************
  * @function		-	CAN_MessagePending
  *
  * @brief			-	This Function Returns the number of pending received messages.
  *
  * @param[in]    	-	CANx: where x can be 1 or 2 to to select the CAN peripheral.
  *
  * @param[in]    	-	FIFONumber: Receive FIFO number, CAN_FIFO0 or CAN_FIFO1.
  *
  * @retval			-	NbMessage : which is the number of pending message.
  *
  * @Note			-	None
  */
uint8_t CAN_MessagePending(CAN_RegDef_t* CANx, uint8_t FIFONumber)
{
	uint8_t message_pending=0;
	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	assert_param(IS_CAN_FIFO(FIFONumber));

	if(FIFONumber == CAN_FIFO0)
	{
		message_pending = (uint8_t)(CANx->RF0R & (uint32_t)0x03);
	}else if (FIFONumber == CAN_FIFO1)
	{
		message_pending = (uint8_t)(CANx->RF1R & (uint32_t)0x03);
	}else
	{
		message_pending = 0;
	}

	return message_pending;
}

/*
===============================================================================
                    ##### CAN Operation modes functions #####
 ===============================================================================
    [..] This section provides functions allowing to select the CAN Operation modes
      (+) sleep mode
      (+) normal mode
      (+) initialization mode
*/

/****************************************************************************************************
  * @function		-	CAN_OperatingModeRequest
  *
  * @brief			-	This Function Selects the CAN Operation mode.
  *
  * @param[in]    	-	CANx: where x can be 1 or 2 to to select the CAN peripheral.
  *
  * @param[in]    	-	CAN_OperatingMode: CAN Operating Mode.
  * 					CAN_OperatingMode_Initialization
  * 					CAN_OperatingMode_Normal
  * 					CAN_OperatingMode_Sleep
  *
  * @retval			-	status of the requested mode which can be:
  * 						CAN_ModeStatus_Failed: CAN failed entering the specific mode
  * 						CAN_ModeStatus_Success: CAN Succeed entering the specific mode
  * @Note			-	None
  */
uint8_t CAN_OperatingModeRequest(CAN_RegDef_t* CANx, uint8_t CAN_OperatingMode)
{
	uint8_t status = CAN_ModeStatus_Failed;
	/* Timeout for INAK or also for SLAK bits*/
	uint32_t timeout = INAK_TIMEOUT;

	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	assert_param(IS_CAN_OPERATING_MODE(CAN_OperatingMode));

	if(CAN_OperatingMode == CAN_OperatingMode_Initialization)
	{
		/* Request initialization */
		CANx->MCR = (uint32_t)(((CANx->MCR) & (uint32_t)(~(uint32_t)CAN_MCR_SLEEP)) | CAN_MCR_INRQ);

		/* Wait the acknowledge */
		while( ((CANx->MSR & CAN_MODE_MASK) != CAN_MSR_INAK) && (timeout != 0) )
		{
			timeout--;
		}
		if ((CANx->MSR & CAN_MODE_MASK) != CAN_MSR_INAK)
		{
		  status = CAN_ModeStatus_Failed;
		}
		else
		{
		  status = CAN_ModeStatus_Success;
		}
	}else if(CAN_OperatingMode == CAN_OperatingMode_Normal)
	{
		/* Normal mode */
		/* Request leave initialization and sleep mode  and enter Normal mode */
		CANx->MCR = (uint32_t)(~(CAN_MCR_SLEEP | CAN_MCR_INRQ));
		/* Wait the acknowledge */
		while( (((CANx->MSR) & CAN_MODE_MASK) != 0) && (timeout != 0) )
		{
			timeout--;
		}
		if ((CANx->MSR & CAN_MODE_MASK) != 0)
		{
		  status = CAN_ModeStatus_Failed;
		}
		else
		{
		  status = CAN_ModeStatus_Success;
		}

	}else if(CAN_OperatingMode == CAN_OperatingMode_Sleepde)
	{
		/* Request Sleep mode */
		CANx->MCR = (uint32_t)(((CANx->MCR) & (uint32_t)(~(uint32_t)CAN_MCR_INRQ)) | CAN_MCR_SLEEP);
		/* Wait the acknowledge */
		while( ((CANx->MSR & CAN_MODE_MASK) != CAN_MSR_SLAK) && (timeout != 0) )
		{
			timeout--;
		}
		if ((CANx->MSR & CAN_MODE_MASK) != CAN_MSR_SLAK)
		{
		  status = CAN_ModeStatus_Failed;
		}
		else
		{
		  status = CAN_ModeStatus_Success;
		}
	}else
	{
		status = CAN_ModeStatus_Failed;
	}

	return status;

}

/****************************************************************************************************
  * @function		-	CAN_Sleep
  *
  * @brief			-	This Function Enters the Sleep (low power) mode.
  *
  * @param[in]    	-	CANx: where x can be 1 or 2 to to select the CAN peripheral.
  *
  * @retval			-	CAN_Sleep_Ok if sleep entered, CAN_Sleep_Failed otherwise.
  *
  * @Note			-	None
  */

uint8_t CAN_Sleep(CAN_RegDef_t* CANx)
{
	uint8_t sleepstatus = CAN_Sleep_Failed;
	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));

	/* Request Sleep mode */
	CANx->MCR = (uint32_t)(((CANx->MCR) & (uint32_t)(~(uint32_t)CAN_MCR_INRQ)) | CAN_MCR_SLEEP);

	/* Sleep mode status */
	if ((CANx->MSR & (CAN_MSR_SLAK | CAN_MSR_INAK)) == CAN_MSR_SLAK)
	{
	  status = CAN_Sleep_Failed;
	}
	else
	{
	  status = CAN_Sleep_Ok;
	}

	/* return sleep mode status */
	return (uint8_t)sleepstatus;
}

/****************************************************************************************************
  * @function		-	CAN_WakeUp
  *
  * @brief			-	This Function Wakes up the CAN peripheral from sleep mode .
  *
  * @param[in]    	-	CANx: where x can be 1 or 2 to to select the CAN peripheral.
  *
  * @retval			-	CAN_WakeUp_Ok if sleep mode left, CAN_WakeUp_Failed otherwise.
  *
  * @Note			-	None
  */
uint8_t CAN_WakeUp(CAN_RegDef_t* CANx)
{
	uint8_t wakeupstatus = CAN_Sleep_Failed;
	uint32_t wait_slak = SLAK_TIMEOUT;
	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	CANx->MCR &= ~(uint32_t)CAN_MCR_SLEEP;
	/* Sleep mode status */
	while(((CANx->MSR & CAN_MSR_SLAK) == CAN_MSR_SLAK)&&(wait_slak != 0))
	{
	wait_slak--;
	}
	if((CANx->MSR & CAN_MSR_SLAK) != CAN_MSR_SLAK)
	{
	/* wake up done : Sleep mode exited */
	wakeupstatus = CAN_WakeUp_Ok;
	}
	/* return wakeup status */
	return (uint8_t)wakeupstatus;
}

/*
===============================================================================
                ##### CAN Bus Error management functions #####
 ===============================================================================
    [..] This section provides functions allowing to
      (+) Return the CANx's last error code (LEC)
      (+) Return the CANx Receive Error Counter (REC)
      (+) Return the LSB of the 9-bit CANx Transmit Error Counter(TEC).

      -@- If TEC is greater than 255, The CAN is in bus-off state.
      -@- if REC or TEC are greater than 96, an Error warning flag occurs.
      -@- if REC or TEC are greater than 127, an Error Passive Flag occurs.
*/

/****************************************************************************************************
  * @function		-	CAN_GetLastErrorCode
  *
  * @brief			-	This Function Returns the CANx's last error code (LEC).
  *
  * @param[in]    	-	CANx: where x can be 1 or 2 to to select the CAN peripheral.
  *
  * @retval			-	Error code:
  *         			 - CAN_ERRORCODE_NoErr: No Error
  *         			 - CAN_ERRORCODE_StuffErr: Stuff Error
  *         			 - CAN_ERRORCODE_FormErr: Form Error
  *         			 - CAN_ERRORCODE_ACKErr : Acknowledgment Error
  *        				 - CAN_ERRORCODE_BitRecessiveErr: Bit Recessive Error
  *         			 - CAN_ERRORCODE_BitDominantErr: Bit Dominant Error
  *        				 - CAN_ERRORCODE_CRCErr: CRC Error
  *         			 - CAN_ERRORCODE_SoftwareSetErr: Software Set Error
  *
  * @Note			-	None
  */
uint8_t CAN_GetLastErrorCode(CAN_RegDef_t* CANx)
{
	uint8_t errorcode = 0;
	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));

	/* Get error code */
	errorcode = ((uint8_t)(CANx->ESR) & (uint8_t)CAN_ESR_LEC);

	/* Return the error code*/
	return errorcode;
}

/****************************************************************************************************
  * @function		-	CAN_GetReceiveErrorCounter
  *
  * @brief			-	This Function Returns the CANx Receive Error Counter (REC).
  *
  * @param[in]    	-	CANx: where x can be 1 or 2 to to select the CAN peripheral.
  *
  * @retval			-	CAN Receive Error Counter.
  *
  * @Note			-	In case of an error during reception, this counter is incremented
  *        				by 1 or by 8 depending on the error condition as defined by the CAN
  *         			standard. After every successful reception, the counter is
  *         			decremented by 1 or reset to 120 if its value was higher than 128.
  *         			When the counter value exceeds 127, the CAN controller enters the
  *         			error passive state.
  */
uint8_t CAN_GetReceiveErrorCounter(CAN_RegDef_t* CANx)
{
	unit8_t counter = 0;
	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));

	/* Get receive Error Counter */
	counter = (uint8_t)((CANx->ESR & CAN_ESR_REC) >> 24);

	/* Return the Receive Error Counter */
	return counter;
}

/****************************************************************************************************
  * @function		-	CAN_GetLSBTransmitErrorCounter
  *
  * @brief			-	This Function Returns the LSB of the 9-bit CANx Transmit Error Counter(TEC).
  *
  * @param[in]    	-	CANx: where x can be 1 or 2 to to select the CAN peripheral.
  *
  * @retval			-	LSB of the 9-bit CAN Transmit Error Counter.
  *
  * @Note			-	None
  */
uint8_t CAN_GetLSBTransmitErrorCounter(CAN_RegDef_t* CANx)
{
	unit8_t counter = 0;
	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));

	/* Get LSB of the 9-bit CANx Transmit Error Counter(TEC) */
	counter = (uint8_t)((CANx->ESR & CAN_ESR_TEC) >> 16);

	/* Return the LSB of the 9-bit CANx Transmit Error Counter(TEC) */
	return counter;
}

/*
===============================================================================
              ##### Interrupts and flags management functions #####
 ===============================================================================

     [..] This section provides functions allowing to configure the CAN Interrupts
          and to get the status and clear flags and Interrupts pending bits.

          The CAN provides 14 Interrupts sources and 15 Flags:


  *** Flags ***
  =============
    [..] The 15 flags can be divided on 4 groups:

      (+) Transmit Flags
        (++) CAN_FLAG_RQCP0,
        (++) CAN_FLAG_RQCP1,
        (++) CAN_FLAG_RQCP2  : Request completed MailBoxes 0, 1 and 2  Flags
                               Set when when the last request (transmit or abort)
                               has been performed.

      (+) Receive Flags


        (++) CAN_FLAG_FMP0,
        (++) CAN_FLAG_FMP1   : FIFO 0 and 1 Message Pending Flags
                               set to signal that messages are pending in the receive
                               FIFO.
                               These Flags are cleared only by hardware.

        (++) CAN_FLAG_FF0,
        (++) CAN_FLAG_FF1    : FIFO 0 and 1 Full Flags
                               set when three messages are stored in the selected
                               FIFO.

        (++) CAN_FLAG_FOV0
        (++) CAN_FLAG_FOV1   : FIFO 0 and 1 Overrun Flags
                               set when a new message has been received and passed
                               the filter while the FIFO was full.

      (+) Operating Mode Flags

        (++) CAN_FLAG_WKU    : Wake up Flag
                               set to signal that a SOF bit has been detected while
                               the CAN hardware was in Sleep mode.

        (++) CAN_FLAG_SLAK   : Sleep acknowledge Flag
                               Set to signal that the CAN has entered Sleep Mode.

      (+) Error Flags

        (++) CAN_FLAG_EWG    : Error Warning Flag
                               Set when the warning limit has been reached (Receive
                               Error Counter or Transmit Error Counter greater than 96).
                               This Flag is cleared only by hardware.

        (++) CAN_FLAG_EPV    : Error Passive Flag
                               Set when the Error Passive limit has been reached
                               (Receive Error Counter or Transmit Error Counter
                               greater than 127).
                               This Flag is cleared only by hardware.

        (++) CAN_FLAG_BOF    : Bus-Off Flag
                               set when CAN enters the bus-off state. The bus-off
                               state is entered on TEC overflow, greater than 255.
                               This Flag is cleared only by hardware.

        (++) CAN_FLAG_LEC    : Last error code Flag
                               set If a message has been transferred (reception or
                               transmission) with error, and the error code is hold.

  *** Interrupts ***
  ==================
    [..] The 14 interrupts can be divided on 4 groups:

      (+) Transmit interrupt

        (++) CAN_IT_TME   :  Transmit mailbox empty Interrupt
                             if enabled, this interrupt source is pending when
                             no transmit request are pending for Tx mailboxes.

      (+) Receive Interrupts

        (++) CAN_IT_FMP0,
        (++) CAN_IT_FMP1    :  FIFO 0 and FIFO1 message pending Interrupts
                               if enabled, these interrupt sources are pending
                               when messages are pending in the receive FIFO.
                               The corresponding interrupt pending bits are cleared
                               only by hardware.

        (++) CAN_IT_FF0,
        (++) CAN_IT_FF1     :  FIFO 0 and FIFO1 full Interrupts
                               if enabled, these interrupt sources are pending
                               when three messages are stored in the selected FIFO.

        (++) CAN_IT_FOV0,
        (++) CAN_IT_FOV1    :  FIFO 0 and FIFO1 overrun Interrupts
                               if enabled, these interrupt sources are pending
                               when a new message has been received and passed
                               the filter while the FIFO was full.

      (+) Operating Mode Interrupts

        (++) CAN_IT_WKU     :  Wake-up Interrupt
                               if enabled, this interrupt source is pending when
                               a SOF bit has been detected while the CAN hardware
                               was in Sleep mode.

        (++) CAN_IT_SLK     :  Sleep acknowledge Interrupt
                               if enabled, this interrupt source is pending when
                               the CAN has entered Sleep Mode.

      (+) Error Interrupts

        (++) CAN_IT_EWG     :  Error warning Interrupt
                               if enabled, this interrupt source is pending when
                               the warning limit has been reached (Receive Error
                               Counter or Transmit Error Counter=96).

        (++) CAN_IT_EPV     :  Error passive Interrupt
                               if enabled, this interrupt source is pending when
                               the Error Passive limit has been reached (Receive
                               Error Counter or Transmit Error Counter>127).

        (++) CAN_IT_BOF     :  Bus-off Interrupt
                               if enabled, this interrupt source is pending when
                               CAN enters the bus-off state. The bus-off state is
                               entered on TEC overflow, greater than 255.
                               This Flag is cleared only by hardware.

        (++) CAN_IT_LEC     :  Last error code Interrupt
                               if enabled, this interrupt source is pending  when
                               a message has been transferred (reception or
                               transmission) with error, and the error code is hold.

        (++) CAN_IT_ERR     :  Error Interrupt
                               if enabled, this interrupt source is pending when
                               an error condition is pending.

    [..] Managing the CAN controller events :

         The user should identify which mode will be used in his application to
         manage the CAN controller events: Polling mode or Interrupt mode.

      (#) In the Polling Mode it is advised to use the following functions:
        (++) CAN_GetFlagStatus() : to check if flags events occur.
        (++) CAN_ClearFlag()     : to clear the flags events.



      (#) In the Interrupt Mode it is advised to use the following functions:
        (++) CAN_ITConfig()       : to enable or disable the interrupt source.
        (++) CAN_GetITStatus()    : to check if Interrupt occurs.
        (++) CAN_ClearITPendingBit() : to clear the Interrupt pending Bit
            (corresponding Flag).
        -@@-  This function has no impact on CAN_IT_FMP0 and CAN_IT_FMP1 Interrupts
             pending bits since there are cleared only by hardware.
*/

/****************************************************************************************************
 * @function		-	CAN_ITConfig
 *
 * @brief			-	This Function Checks the transmission status of a CAN Frame.
 *
 * @param[in]    	-	CANx: where x can be 1 or 2 to to select the CAN peripheral.
 *
 * @param[in]    	-	CAN_IT: specifies the CAN interrupt sources to be enabled or disabled.
 *
 * 						This parameter can be:
 * 							 @arg CAN_IT_TME: Transmit mailbox empty Interrupt
 *							 @arg CAN_IT_TME: Transmit mailbox empty Interrupt
 *           			     @arg CAN_IT_FMP0: FIFO 0 message pending Interrupt
 *          				 @arg CAN_IT_FF0: FIFO 0 full Interrupt
 *          				 @arg CAN_IT_FOV0: FIFO 0 overrun Interrupt
 *          				 @arg CAN_IT_FMP1: FIFO 1 message pending Interrupt
 *          				 @arg CAN_IT_FF1: FIFO 1 full Interrupt
 *           				 @arg CAN_IT_FOV1: FIFO 1 overrun Interrupt
 *          				 @arg CAN_IT_WKU: Wake-up Interrupt
 *          				 @arg CAN_IT_SLK: Sleep acknowledge Interrupt
 *           			 	 @arg CAN_IT_EWG: Error warning Interrupt
 *           				 @arg CAN_IT_EPV: Error passive Interrupt
 *          				 @arg CAN_IT_BOF: Bus-off Interrupt
 *           				 @arg CAN_IT_LEC: Last error code Interrupt
 *          				 @arg CAN_IT_ERR: Error Interrupt
 *
 * @param[in]    	-	NewState: new state of the CAN interrupts. ENABLE or DISABLE.
 *
 * @retval			-	None
 *
 * @Note			-	None
 */

void CAN_ITConfig(CAN_RegDef_t* CANx, uint32_t CAN_IT, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	assert_param(IS_CAN_IT(CAN_IT));
	assert_param(IS_FUNCTIONAL_STATE(NewState));

	if(NewState != DISABLE)
	{
		CANx->IER |= CAN_IT;
	}else
	{
		CANx->IER &= ~CAN_IT;
	}
}

/****************************************************************************************************
 * @function		-	CAN_GetFlagStatus
 *
 * @brief			-	This Function Checks whether the specified CAN flag is set or not.
 *
 * @param[in]    	-	CANx: where x can be 1 or 2 to to select the CAN peripheral.
 *
 * @param[in]    	-	CAN_FLAG: specifies the flag to check.
 *
 * 						This parameter can be:
 *            				 @arg CAN_FLAG_RQCP0: Request MailBox0 Flag
 *           				 @arg CAN_FLAG_RQCP1: Request MailBox1 Flag
 *            				 @arg CAN_FLAG_RQCP2: Request MailBox2 Flag
 *           				 @arg CAN_FLAG_FMP0: FIFO 0 Message Pending Flag
 *           				 @arg CAN_FLAG_FF0: FIFO 0 Full Flag
 *           				 @arg CAN_FLAG_FOV0: FIFO 0 Overrun Flag
 *           				 @arg CAN_FLAG_FMP1: FIFO 1 Message Pending Flag
 *           				 @arg CAN_FLAG_FF1: FIFO 1 Full Flag
 *            				 @arg CAN_FLAG_FOV1: FIFO 1 Overrun Flag
 *         			   	 	 @arg CAN_FLAG_WKU: Wake up Flag
 *            				 @arg CAN_FLAG_SLAK: Sleep acknowledge Flag
 *           				 @arg CAN_FLAG_EWG: Error Warning Flag
 *           				 @arg CAN_FLAG_EPV: Error Passive Flag
 *           				 @arg CAN_FLAG_BOF: Bus-Off Flag
 *           				 @arg CAN_FLAG_LEC: Last error code Flag
 * @retval			-	The new state of CAN_FLAG (SET or RESET).
 *
 * @Note			-	None
 */

FlagStatus CAN_GetFlagStatus(CAN_RegDef_t* CANx, uint32_t CAN_FLAG)
{
	FlagStatus bitstatus = RESET;

	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	assert_param(IS_CAN_GET_FLAG(CAN_FLAG));

	/* Flags in ESR register */
	if((CAN_FLAG & CAN_FLAGS_ESR) != (uint32_t)RESET)
	{
		/* Check the status of the specified CAN flag */
		if((CANx->ESR & (CAN_FLAG & 0x000FFFFF)) != (uint32_t)RESET)
		{
			/* CAN_FLAG is set */
			bitstatus = SET;
		}else
		{
			/* CAN_FLAG is reset */
			bitstatus = RESET;
		}

	}else if((CAN_FLAG & CAN_FLAGS_MSR) != (uint32_t)RESET)
	{
		/* Check the status of the specified CAN flag */
		if((CANx->MSR & (CAN_FLAG & 0x000FFFFF)) != (uint32_t)RESET)
		{
			/* CAN_FLAG is set */
			bitstatus = SET;
		}else
		{
			/* CAN_FLAG is reset */
			bitstatus = RESET;
		}
	}else if((CAN_FLAG & CAN_FLAGS_TSR) != (uint32_t)RESET)
	{
		/* Check the status of the specified CAN flag */
		if((CANx->TSR & (CAN_FLAG & 0x000FFFFF)) != (uint32_t)RESET)
		{
			/* CAN_FLAG is set */
			bitstatus = SET;
		}else
		{
			/* CAN_FLAG is reset */
			bitstatus = RESET;
		}
	}else if((CAN_FLAG & CAN_FLAGS_RF0R) != (uint32_t)RESET)
	{
		/* Check the status of the specified CAN flag */
		if((CANx->RF0R & (CAN_FLAG & 0x000FFFFF)) != (uint32_t)RESET)
		{
			/* CAN_FLAG is set */
			bitstatus = SET;
		}else
		{
			/* CAN_FLAG is reset */
			bitstatus = RESET;
		}
	}else /* if((CAN_FLAG & CAN_FLAGS_RF1R) != (uint32_t)RESET) */
	{
		/* Check the status of the specified CAN flag */
		if((CANx->RF1R & (CAN_FLAG & 0x000FFFFF)) != (uint32_t)RESET)
		{
			/* CAN_FLAG is set */
			bitstatus = SET;
		}else
		{
			/* CAN_FLAG is reset */
			bitstatus = RESET;
		}
	}
	/* Return the CAN_FLAG status */
	return  bitstatus;
}

/****************************************************************************************************
 * @function		-	CAN_ClearFlag
 *
 * @brief			-	This Function Clears the CAN's pending flags.
 *
 * @param[in]    	-	CANx: where x can be 1 or 2 to to select the CAN peripheral.
 *
 * @param[in]    	-	CAN_FLAG: specifies the flag to check.
 *
 * 						This parameter can be:
 *            				 @arg CAN_FLAG_RQCP0: Request MailBox0 Flag
 *           				 @arg CAN_FLAG_RQCP1: Request MailBox1 Flag
 *            				 @arg CAN_FLAG_RQCP2: Request MailBox2 Flag
 *           				 @arg CAN_FLAG_FMP0: FIFO 0 Message Pending Flag
 *           				 @arg CAN_FLAG_FF0: FIFO 0 Full Flag
 *           				 @arg CAN_FLAG_FOV0: FIFO 0 Overrun Flag
 *           				 @arg CAN_FLAG_FMP1: FIFO 1 Message Pending Flag
 *           				 @arg CAN_FLAG_FF1: FIFO 1 Full Flag
 *            				 @arg CAN_FLAG_FOV1: FIFO 1 Overrun Flag
 *         			   	 	 @arg CAN_FLAG_WKU: Wake up Flag
 *            				 @arg CAN_FLAG_SLAK: Sleep acknowledge Flag
 *           				 @arg CAN_FLAG_EWG: Error Warning Flag
 *           				 @arg CAN_FLAG_EPV: Error Passive Flag
 *           				 @arg CAN_FLAG_BOF: Bus-Off Flag
 *           				 @arg CAN_FLAG_LEC: Last error code Flag
 * @retval			-	None
 *
 * @Note			-	None
 */
void CAN_ClearFlag(CAN_RegDef_t* CANx, uint32_t CAN_FLAG)
{
	uint32_t flagtmp=0;
	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	assert_param(IS_CAN_CLEAR_FLAG(CAN_FLAG));
	if(CAN_FLAG == CAN_FLAG_LEC) /* ESR register */
	{
		/* Clear the selected CAN flags */
		CANx->ESR = (uint32_t)RESET;
	}else /* MSR or TSR or RF0R or RF1R */
	{
		flagtmp = CAN_FLAG & 0x000FFFFF;
		if ((CAN_FLAG & CAN_FLAGS_RF0R)!=(uint32_t)RESET)
		{
			/* Receive Flags */
			CANx->RF0R = (uint32_t)(flagtmp);
		}else if((CAN_FLAG & CAN_FLAGS_RF01R)!=(uint32_t)RESET)
		{
			/* Receive Flags */
			CANx->RF1R = (uint32_t)(flagtmp);
		}else if((CAN_FLAG & CAN_FLAGS_TSR)!=(uint32_t)RESET)
		{
			/* Transmit Flags */
			CANx->TSR = (uint32_t)(flagtmp);
		}else /* if((CAN_FLAG & CAN_FLAGS_MSR)!=(uint32_t)RESET) */
		{
			/* Operating mode Flags */
			CANx->MSR = (uint32_t)(flagtmp);
		}
	}
}

/****************************************************************************************************
 * @function		-	CAN_GetITStatus
 *
 * @brief			-	This Function CChecks whether the specified CANx interrupt has occurred or not.
 *
 * @param  			-	CAN_IT: specifies the CAN interrupt source to check.
 *          			This parameter can be one of the following values:
 *           			 @arg CAN_IT_TME: Transmit mailbox empty Interrupt
 *            			 @arg CAN_IT_FMP0: FIFO 0 message pending Interrupt
 *            			 @arg CAN_IT_FF0: FIFO 0 full Interrupt
 *           			 @arg CAN_IT_FOV0: FIFO 0 overrun Interrupt
 *          			 @arg CAN_IT_FMP1: FIFO 1 message pending Interrupt
 *           			 @arg CAN_IT_FF1: FIFO 1 full Interrupt
 *           			 @arg CAN_IT_FOV1: FIFO 1 overrun Interrupt
 *           			 @arg CAN_IT_WKU: Wake-up Interrupt
 *           			 @arg CAN_IT_SLK: Sleep acknowledge Interrupt
 *           			 @arg CAN_IT_EWG: Error warning Interrupt
 *           			 @arg CAN_IT_EPV: Error passive Interrupt
 *           			 @arg CAN_IT_BOF: Bus-off Interrupt
 *           			 @arg CAN_IT_LEC: Last error code Interrupt
 *            			 @arg CAN_IT_ERR: Error Interrupt
 *
 * @retval 			-	The current state of CAN_IT (SET or RESET).
 *
 * @Note			-	None
 */

ITStatus CAN_GetITStatus(CAN_RegDef_t* CANx, uint32_t CAN_IT)
{
	ITStatus itstatus = RESET;
	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	assert_param(IS_CAN_IT(CAN_IT));
	/* check the interrupt enable bit */
	if((CANx->IER & CAN_IT) != RESET)
	{
		/* in case the Interrupt is enabled, .... */
		switch(CAN_IT)
		{
			case CAN_IT_TME:
				/* Check CAN_TSR_RQCPx bits */
				itstatus = CheckITStatus(CANx->TSR, CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2);
				break;
			case CAN_IT_FMP0:
				/* Check CAN_RF0R_FMP0 bit */
				itstatus = CheckITStatus(CANx->RF0R, CAN_RF0R_FMP0);
				break;
			case CAN_IT_FF0:
				/* Check CAN_RF0R_FULL0 bit */
				itstatus = CheckITStatus(CANx->RF0R, CAN_RF0R_FULL0);
				break;
			case CAN_IT_FOV0:
				/* Check CAN_RF0R_FOVR0 bit */
				itstatus = CheckITStatus(CANx->RF0R, CAN_RF0R_FOVR0);
				break;
			case CAN_IT_FMP1:
				/* Check CAN_RF1R_FMP1 bit */
				itstatus = CheckITStatus(CANx->RF1R, CAN_RF1R_FMP1);
				break;
			case CAN_IT_FF1:
				/* Check CAN_RF1R_FULL1 bit */
				itstatus = CheckITStatus(CANx->RF1R, CAN_RF1R_FMP1);
				break;
			case CAN_IT_FOV1:
				/* Check CAN_RF1R_FOVR1 bit */
				itstatus = CheckITStatus(CANx->RF1R, CAN_RF1R_FOVR1);
				break;
			case CAN_IT_WKU:
				/* Check CAN_MSR_WKUI bit */
				itstatus = CheckITStatus(CANx->MSR, CAN_MSR_WKUI);
				break;
			case CAN_IT_SLK:
				/* Check CAN_MSR_SLAKI bit */
				itstatus = CheckITStatus(CANx->MSR, CAN_MSR_SLAKI);
				break;
			case CAN_IT_EWG:
				/* Check CAN_ESR_EWGF bit */
				itstatus = CheckITStatus(CANx->ESR, CAN_ESR_EWGF);
				break;
			case CAN_IT_EPV:
				/* Check CAN_ESR_EPVF bit */
				itstatus = CheckITStatus(CANx->ESR, CAN_ESR_EPVF);
				break;
			case CAN_IT_BOF:
				/* Check CAN_ESR_BOFF bit */
				itstatus = CheckITStatus(CANx->ESR, CAN_ESR_BOFF);
				break;
			case CAN_IT_LEC:
				/* Check CAN_ESR_LEC bit */
				itstatus = CheckITStatus(CANx->ESR, CAN_ESR_LEC);
				break;
			case CAN_IT_ERR:
				/* Check CAN_MSR_ERRI bit */
				itstatus = CheckITStatus(CANx->MSR, CAN_MSR_ERRI);
				break;
			default:
				/* in case of error, return RESET */
				itstatus = RESET;
				break;
		}
	}else
	{
		/* in case the Interrupt is not enabled, return RESET */
		itstatus  = RESET;
	}
	/* Return the CAN_IT status */
	return itstatus;
}

/****************************************************************************************************
 * @function		-	CAN_ClearITPendingBit
 *
 * @brief			-	This Function Clears the CANx's interrupt pending bits.
 *
 * @param[in]    	-	CANx: where x can be 1 or 2 to to select the CAN peripheral.
 *
 * @param[in]    	-	CAN_IT: specifies the interrupt pending bit to clear.
 * 						This parameter can be one of the following values:
 *            			@arg CAN_IT_TME: Transmit mailbox empty Interrupt
 *            			@arg CAN_IT_FF0: FIFO 0 full Interrupt
 *            			@arg CAN_IT_FOV0: FIFO 0 overrun Interrupt
 *            			@arg CAN_IT_FF1: FIFO 1 full Interrupt
 *            			@arg CAN_IT_FOV1: FIFO 1 overrun Interrupt
 *            			@arg CAN_IT_WKU: Wake-up Interrupt
 *            			@arg CAN_IT_SLK: Sleep acknowledge Interrupt
 *            			@arg CAN_IT_EWG: Error warning Interrupt
 *            			@arg CAN_IT_EPV: Error passive Interrupt
 *            			@arg CAN_IT_BOF: Bus-off Interrupt
 *            			@arg CAN_IT_LEC: Last error code Interrupt
 *            			@arg CAN_IT_ERR: Error Interrupt
 *
 * @retval			-	None
 *
 * @Note			-	None
 */

void CAN_ClearITPendingBit(CAN_RegDef_t* CANx, uint32_t CAN_IT)
{
	/* Check the parameters */
	  assert_param(IS_CAN_ALL_PERIPH(CANx));
	  assert_param(IS_CAN_CLEAR_IT(CAN_IT));
	  switch(CAN_IT)
	  {
		case CAN_IT_TME:
			/* Clear CAN_TSR_RQCPx (rc_w1). Cleared by software writing a “1” */
			CANx->TSR = CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2;
			break;
		case CAN_IT_FF0:
			/* Clear CAN_RF0R_FULL0 (rc_w1)*/
			CANx->RF0R = CAN_RF0R_FULL0;
			break;
		case CAN_IT_FOV0:
			/* Clear CAN_RF0R_FOVR0 (rc_w1)*/
			CANx->RF0R = CAN_RF0R_FOVR0;
			break;
		case CAN_IT_FF1:
			/* Clear CAN_RF1R_FULL1 (rc_w1)*/
			CANx->RF1R = CAN_RF1R_FULL1;
			break;
		case CAN_IT_FOV1:
			/* Clear CAN_RF1R_FOVR1 (rc_w1)*/
			CANx->RF1R = CAN_RF1R_FOVR1;
			break;
		case CAN_IT_WKU:
			/* Clear CAN_MSR_WKUI (rc_w1)*/
			CANx->MSR = CAN_MSR_WKUI;
			break;
		case CAN_IT_SLK:
			/* Clear CAN_MSR_SLAKI (rc_w1)*/
			CANx->MSR = CAN_MSR_SLAKI;
			break;
		case CAN_IT_EWG:
			/* Clear CAN_MSR_ERRI (rc_w1) */
			CANx->MSR = CAN_MSR_ERRI;
			/* @note the corresponding Flag is cleared by hardware depending on the CAN Bus status*/
			break;
		case CAN_IT_EPV:
			/* Clear CAN_MSR_ERRI (rc_w1) */
			CANx->MSR = CAN_MSR_ERRI;
			/* @note the corresponding Flag is cleared by hardware depending on the CAN Bus status*/
			break;
		case CAN_IT_BOF:
			/* Clear CAN_MSR_ERRI (rc_w1) */
			CANx->MSR = CAN_MSR_ERRI;
			/* @note the corresponding Flag is cleared by hardware depending on the CAN Bus status*/
			break;
		case CAN_IT_LEC:
			/*  Clear LEC bits */
			CANx->ESR = RESET;
			/* Clear CAN_MSR_ERRI (rc_w1) */
			CANx->MSR = CAN_MSR_ERRI;
			break;
		case CAN_IT_ERR:
			/*Clear LEC bits */
			CANx->ESR = RESET;
			/* Clear CAN_MSR_ERRI (rc_w1) */
			CANx->MSR = CAN_MSR_ERRI;
			/* @note BOFF, EPVF and EWGF Flags are cleared by hardware depending on the CAN Bus status*/
			break;
		default:
			break;
	  }
}

/****************************************************************************************************
 * @function		-	CheckITStatus
 *
 * @brief			-	This Function Checks whether the CAN interrupt has occurred or not.
 *
 * @param[in]    	-	CANx: where x can be 1 or 2 to to select the CAN peripheral.
 *
 * @param[in]    	-	It_Bit: specifies the interrupt source bit to check.
 *
 * @retval			-	The new state of the CAN Interrupt (SET or RESET).
 *
 * @Note			-	None
 */
static ITStatus CheckITStatus(uint32_t CAN_Reg, uint32_t It_Bit)
{
	ITStatus pendingbitstatus = RESET;
	if((CAN_Reg & It_Bit) != (uint32_t)RESET)
	{
		/* CAN_IT is set */
	pendingbitstatus = SET;
	}else
	{
		/* CAN_IT is reset */
		pendingbitstatus = RESET;
	}

	return pendingbitstatus;
}

/*************************************END OF FILE**************************************************/
