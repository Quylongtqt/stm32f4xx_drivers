/*
 * stm32f407xx_can_driver.h
 *
 *  Created on: Jul 24, 2021
 *      Author: Quy Long
 */

#ifndef INC_STM32F407XX_CAN_DRIVER_H_
#define INC_STM32F407XX_CAN_DRIVER_H_

#define CAN_Mode_Normal				0
#define CAN_Mode_LoopBack			1
#define CAN_Mode_Silent				2
#define CAN_Mode_Silent_LoopBack	3

#define IS_CAN_MODE(MODE) (((MODE) == CAN_Mode_Normal) || \
                           ((MODE) == CAN_Mode_LoopBack)|| \
                           ((MODE) == CAN_Mode_Silent) || \
                           ((MODE) == CAN_Mode_Silent_LoopBack))

/* CAN_synchronisation_jump_width */
#define CAN_SJW_1tq                 ((uint8_t)0x00)  /*!< 1 time quantum */
#define CAN_SJW_2tq                 ((uint8_t)0x01)  /*!< 2 time quantum */
#define CAN_SJW_3tq                 ((uint8_t)0x02)  /*!< 3 time quantum */
#define CAN_SJW_4tq                 ((uint8_t)0x03)  /*!< 4 time quantum */

#define IS_CAN_SJW(SJW) (((SJW) == CAN_SJW_1tq) || ((SJW) == CAN_SJW_2tq)|| \
                         ((SJW) == CAN_SJW_3tq) || ((SJW) == CAN_SJW_4tq))

/* CAN_time_quantum_in_bit_segment_1 */
#define CAN_BS1_1tq                 ((uint8_t)0x00)  /*!< 1 time quantum */
#define CAN_BS1_2tq                 ((uint8_t)0x01)  /*!< 2 time quantum */
#define CAN_BS1_3tq                 ((uint8_t)0x02)  /*!< 3 time quantum */
#define CAN_BS1_4tq                 ((uint8_t)0x03)  /*!< 4 time quantum */
#define CAN_BS1_5tq                 ((uint8_t)0x04)  /*!< 5 time quantum */
#define CAN_BS1_6tq                 ((uint8_t)0x05)  /*!< 6 time quantum */
#define CAN_BS1_7tq                 ((uint8_t)0x06)  /*!< 7 time quantum */
#define CAN_BS1_8tq                 ((uint8_t)0x07)  /*!< 8 time quantum */
#define CAN_BS1_9tq                 ((uint8_t)0x08)  /*!< 9 time quantum */
#define CAN_BS1_10tq                ((uint8_t)0x09)  /*!< 10 time quantum */
#define CAN_BS1_11tq                ((uint8_t)0x0A)  /*!< 11 time quantum */
#define CAN_BS1_12tq                ((uint8_t)0x0B)  /*!< 12 time quantum */
#define CAN_BS1_13tq                ((uint8_t)0x0C)  /*!< 13 time quantum */
#define CAN_BS1_14tq                ((uint8_t)0x0D)  /*!< 14 time quantum */
#define CAN_BS1_15tq                ((uint8_t)0x0E)  /*!< 15 time quantum */
#define CAN_BS1_16tq                ((uint8_t)0x0F)  /*!< 16 time quantum */

#define IS_CAN_BS1(BS1) ((BS1) <= CAN_BS1_16tq)

/* CAN_time_quantum_in_bit_segment_2 */
#define CAN_BS2_1tq                 ((uint8_t)0x00)  /*!< 1 time quantum */
#define CAN_BS2_2tq                 ((uint8_t)0x01)  /*!< 2 time quantum */
#define CAN_BS2_3tq                 ((uint8_t)0x02)  /*!< 3 time quantum */
#define CAN_BS2_4tq                 ((uint8_t)0x03)  /*!< 4 time quantum */
#define CAN_BS2_5tq                 ((uint8_t)0x04)  /*!< 5 time quantum */
#define CAN_BS2_6tq                 ((uint8_t)0x05)  /*!< 6 time quantum */
#define CAN_BS2_7tq                 ((uint8_t)0x06)  /*!< 7 time quantum */
#define CAN_BS2_8tq                 ((uint8_t)0x07)  /*!< 8 time quantum */

#define IS_CAN_BS2(BS2) ((BS2) <= CAN_BS2_8tq)

/* CAN_clock_prescaler */
#define IS_CAN_PRESCALER(PRESCALER) (((PRESCALER) >= 1) && ((PRESCALER) <= 1024))

/* Is CAN peripheral */
#define IS_CAN_ALL_PERIPH(PERIPH) (((PERIPH) == CAN1) || \
                                   ((PERIPH) == CAN2))

/* CAN_filter_number */
#define IS_CAN_FILTER_NUMBER(NUMBER) ((NUMBER) <= 27)

/* CAN_filter_mode */
#define CAN_FilterMode_IdMask       ((uint8_t)0x00)  /*!< identifier/mask mode */
#define CAN_FilterMode_IdList       ((uint8_t)0x01)  /*!< identifier list mode */

#define IS_CAN_FILTER_MODE(MODE) (((MODE) == CAN_FilterMode_IdMask) || \
                                  ((MODE) == CAN_FilterMode_IdList))


/* CAN_filter_scale */
#define CAN_FilterScale_16bit       ((uint8_t)0x00) /*!< Two 16-bit filters */
#define CAN_FilterScale_32bit       ((uint8_t)0x01) /*!< One 32-bit filter */

#define IS_CAN_FILTER_SCALE(SCALE) (((SCALE) == CAN_FilterScale_16bit) || \
                                    ((SCALE) == CAN_FilterScale_32bit))
/* CAN_filter_FIFO */
#define CAN_Filter_FIFO0             ((uint8_t)0x00)  /*!< Filter FIFO 0 assignment for filter x */
#define CAN_Filter_FIFO1             ((uint8_t)0x01)  /*!< Filter FIFO 1 assignment for filter x */
#define IS_CAN_FILTER_FIFO(FIFO) (((FIFO) == CAN_FilterFIFO0) || \
                                  ((FIFO) == CAN_FilterFIFO1))

#define IS_CAN_BANKNUMBER(BANKNUMBER) ((BANKNUMBER >=1) &&(BANKNUMBER <= 27))

#define IS_CAN_TRANSMITMAILBOX(TRANSMITMAILBOX) ((TRANSMITMAILBOX) <= ((uint8_t)0x02))
#define IS_CAN_STDID(STDID)   ((STDID) <= ((uint32_t)0x7FF))
#define IS_CAN_EXTID(EXTID)   ((EXTID) <= ((uint32_t)0x1FFFFFFF))
#define IS_CAN_DLC(DLC)       ((DLC) <= ((uint8_t)0x08))

#define IS_CAN_IDTYPE(IDTYPE) (((IDTYPE) == CAN_Id_Standard) || \
                               ((IDTYPE) == CAN_Id_Extended))

/* Legacy defines */
#define CAN_ID_STD      CAN_Id_Standard
#define CAN_ID_EXT      CAN_Id_Extended

#define CAN_RTR_Data                ((uint32_t)0x00000000)  /*!< Data frame */
#define CAN_RTR_Remote              ((uint32_t)0x00000002)  /*!< Remote frame */
#define IS_CAN_RTR(RTR) (((RTR) == CAN_RTR_Data) || ((RTR) == CAN_RTR_Remote))

/* Legacy defines */
#define CAN_RTR_DATA     CAN_RTR_Data
#define CAN_RTR_REMOTE   CAN_RTR_Remote
/** @defgroup CAN_transmit_constants
  * @{
  */
#define CAN_TxStatus_Failed         ((uint8_t)0x00)/*!< CAN transmission failed */
#define CAN_TxStatus_Ok             ((uint8_t)0x01) /*!< CAN transmission succeeded */
#define CAN_TxStatus_Pending        ((uint8_t)0x02) /*!< CAN transmission pending */
#define CAN_TxStatus_NoMailBox      ((uint8_t)0x04) /*!< CAN cell did not provide
                                                         an empty mailbox */
/* Legacy defines */
#define CANTXFAILED                  CAN_TxStatus_Failed
#define CANTXOK                      CAN_TxStatus_Ok
#define CANTXPENDING                 CAN_TxStatus_Pending
#define CAN_NO_MB                    CAN_TxStatus_NoMailBox

#define CAN_FIFO0                 ((uint8_t)0x00) /*!< CAN FIFO 0 used to receive */
#define CAN_FIFO1                 ((uint8_t)0x01) /*!< CAN FIFO 1 used to receive */

#define IS_CAN_FIFO(FIFO) (((FIFO) == CAN_FIFO0) || ((FIFO) == CAN_FIFO1))

#define CAN_OperatingMode_Initialization  ((uint8_t)0x00) /*!< Initialization mode */
#define CAN_OperatingMode_Normal          ((uint8_t)0x01) /*!< Normal mode */
#define CAN_OperatingMode_Sleep           ((uint8_t)0x02) /*!< sleep mode */

#define IS_CAN_OPERATING_MODE(MODE) (((MODE) == CAN_OperatingMode_Initialization) ||\
                                     ((MODE) == CAN_OperatingMode_Normal)|| \
									 ((MODE) == CAN_OperatingMode_Sleep))
#define CAN_ModeStatus_Failed    ((uint8_t)0x00)                /*!< CAN entering the specific mode failed */
#define CAN_ModeStatus_Success   ((uint8_t)!CAN_ModeStatus_Failed)   /*!< CAN entering the specific mode Succeed */

#define CAN_Sleep_Failed     ((uint8_t)0x00) /*!< CAN did not enter the sleep mode */
#define CAN_Sleep_Ok         ((uint8_t)0x01) /*!< CAN entered the sleep mode */

/* Legacy defines */
#define CANSLEEPFAILED   CAN_Sleep_Failed
#define CANSLEEPOK       CAN_Sleep_Ok

/* CAN_wake_up_constants */
#define CAN_WakeUp_Failed        ((uint8_t)0x00) /*!< CAN did not leave the sleep mode */
#define CAN_WakeUp_Ok            ((uint8_t)0x01) /*!< CAN leaved the sleep mode */

/* Legacy defines */
#define CANWAKEUPFAILED   CAN_WakeUp_Failed
#define CANWAKEUPOK       CAN_WakeUp_Ok

typedef struct
{
	uint16_t		CAN_Prescaler;	/*!< Specifies the length of a time quantum. */
	uint8_t			CAN_Mode;		/*!< Specifies the CAN operating mode. */
	uint8_t			CAN_SJW;		/*!< Specifies the maximum number of time quanta
									 the CAN hardware is allowed to lengthen or
									 shorten a bit to perform re-synchronization. */
	uint8_t			CAN_BS1;		/*!< Specifies the number of time quanta in Bit
									 Segment 1. This parameter can be a value of
									 CAN_time_quantum_in_bit_segment_1 */
	uint8_t			CAN_BS2;		/*!< Specifies the number of time quanta in Bit Segment 2.
                                 	 This parameter can be a value of CAN_time_quantum_in_bit_segment_2 */
	FunctionalState	CAN_TTCM;		/*!< Enable or disable the time triggered communication mode. */
	FunctionalState	CAN_ABOM;		/*!< Enable or disable the automatic bus-off management. */
	FunctionalState	CAN_AWUM;		/*!< Enable or disable the automatic wake-up mode. */
	FunctionalState	CAN_NART;		/*!< Enable or disable the non-automatic retransmission mode. */
	FunctionalState	CAN_RFLM;		/*!< Enable or disable the Receive FIFO Locked mode. */
	FunctionalState	CAN_TXFP;		/*!< Enable or disable the transmit FIFO priority. */
}CAN_Config_t;

typedef struct
{
	CAN_RegDef_t	*pCANx; 	/*!< This holds the based address of CANx(x:1,2) peripheral >*/
	CAN_Config_t	CANConfig;
}CAN_Handle_t;

typedef struct
{
	uint16_t CAN_FilterIdHigh;				/*!< Specifies the filter identification number */
	uint16_t CAN_FilterIdLow;				/*!< Specifies the filter identification number */
	uint16_t CAN_FilterMaskIdHigh;			/*!< Specifies the filter mask number or identification number. */
	uint16_t CAN_FilterMaskIdLow;			/*!< Specifies the filter mask number or identification number. */
	uint16_t CAN_FilterFIFOAssignment;		/*!< Specifies the FIFO (0 or 1) which will be assigned to the filter. */
	uint8_t  CAN_FilterNumber;				/*!< Specifies the filter which will be initialized. It ranges from 0 to 13. */
	uint8_t  CAN_FilterMode;				/*!< Specifies the filter mode to be initialized. */
	uint8_t  CAN_FilterScale;				/*!< Specifies the filter scale. */
	FunctionalState CAN_FilterActivation;	/*!< Enable or disable the filter. */
}CAN_FilterInitTypeDef;

/*
 * CAN Tx message structure definition
 */
typedef struct
{
	uint32_t StdId;  /*!< Specifies the standard identifier.
						This parameter can be a value between 0 to 0x7FF. */
	uint32_t ExtId;  /*!< Specifies the extended identifier.
						This parameter can be a value between 0 to 0x1FFFFFFF. */
	uint8_t IDE;     /*!< Specifies the type of identifier for the message that
						will be transmitted. This parameter can be a value
						of @ref CAN_identifier_type */
	uint8_t RTR;     /*!< Specifies the type of frame for the message that will
						be transmitted. This parameter can be a value of
						@ref CAN_remote_transmission_request */
	uint8_t DLC;     /*!< Specifies the length of the frame that will be
						transmitted. This parameter can be a value between
						0 to 8 */
	uint8_t Data[8]; /*!< Contains the data to be transmitted. It ranges from 0
	                        to 0xFF. */
}CAN_TxMsg;

/*
 *  Rx message structure definition
 */
typedef struct
{
  uint32_t StdId;  /*!< Specifies the standard identifier.
                        This parameter can be a value between 0 to 0x7FF. */
  uint32_t ExtId;  /*!< Specifies the extended identifier.
                        This parameter can be a value between 0 to 0x1FFFFFFF. */
  uint8_t IDE;     /*!< Specifies the type of identifier for the message that
                        will be received. This parameter can be a value of
                        @ref CAN_identifier_type */
  uint8_t RTR;     /*!< Specifies the type of frame for the received message.
                        This parameter can be a value of
                        @ref CAN_remote_transmission_request */
  uint8_t DLC;     /*!< Specifies the length of the frame that will be received.
                        This parameter can be a value between 0 to 8 */
  uint8_t Data[8]; /*!< Contains the data to be received. It ranges from 0 to
                        0xFF. */
  uint8_t FMI;     /*!< Specifies the index of the filter the message stored in
                        the mailbox passes through. This parameter can be a
                        value between 0 to 0xFF */
} CanRxMsg;

/* CAN_InitStatus*/
#define CAN_InitStatus_Failed              0 /*!< CAN initialization failed */
#define CAN_InitStatus_Success             1 /*!< CAN initialization OK */

/*  Function used to set the CAN configuration to the default reset state *****/
void CAN_DeInit (CAN_RegDef_t *CANx);

/* Initialization and Configuration functions *********************************/
void CAN_PeriperalClockControl(CAN_RegDef_t *pCANx, uint8_t ENorDIS);
uint8_t CANInit (CAN_Handle_t *pCANHanlde);
void CAN_FilterInit(CAN_FilterInitTypeDef* CAN_FilterInitStruct);
void CAN_StructInit(CAN_Config_t* CAN_InitStruct);
void CAN_SlaveStartBank(uint8_t CAN_BankNumber);
void CAN_DBGFreeze(CAN_RegDef_t* CANx, FunctionalState NewState);
void CAN_TTComModeCmd(CAN_RegDef_t* CANx, FunctionalState NewState);

/* CAN Frames Transmission functions ******************************************/
uint8_t	CAN_Transmit(CAN_RegDef_t *CANx, CANTxMsg* TxMessage);
uint8_t CAN_TransmitStatus(CAN_RegDef_t* CANx, uint8_t TransmitMailbox);
void CAN_CancelTransmit(CAN_RegDef_t* CANx, uint8_t Mailbox);

/* CAN Frames Reception functions *********************************************/
void CAN_Receive(CAN_RegDef_t* CANx, uint8_t FIFONumber, CanRxMsg* RxMessage);
void CAN_FIFORelease(CAN_RegDef_t* CANx, uint8_t FIFONumber);
uint8_t CAN_MessagePending(CAN_RegDef_t* CANx, uint8_t FIFONumber);

/* Operation modes functions **************************************************/
uint8_t CAN_OperatingModeRequest(CAN_RegDef_t* CANx, uint8_t CAN_OperatingMode);
uint8_t CAN_Sleep(CAN_RegDef_t* CANx);
uint8_t CAN_WakeUp(CAN_RegDef_t* CANx);

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


#endif /* INC_STM32F407XX_CAN_DRIVER_H_ */
