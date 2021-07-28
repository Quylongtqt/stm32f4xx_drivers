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

/* CAN Bus Error management functions *****************************************/
uint8_t CAN_GetLastErrorCode(CAN_RegDef_t* CANx);
uint8_t CAN_GetReceiveErrorCounter(CAN_RegDef_t* CANx);
uint8_t CAN_GetLSBTransmitErrorCounter(CAN_RegDef_t* CANx);

/* Interrupts and flags management functions **********************************/
void CAN_ITConfig(CAN_RegDef_t* CANx, uint32_t CAN_IT, FunctionalState NewState);
FlagStatus CAN_GetFlagStatus(CAN_RegDef_t* CANx, uint32_t CAN_FLAG);
void CAN_ClearFlag(CAN_RegDef_t* CANx, uint32_t CAN_FLAG);
ITStatus CAN_GetITStatus(CAN_RegDef_t* CANx, uint32_t CAN_IT);
void CAN_ClearITPendingBit(CAN_RegDef_t* CANx, uint32_t CAN_IT);
static ITStatus CheckITStatus(uint32_t CAN_Reg, uint32_t It_Bit);

