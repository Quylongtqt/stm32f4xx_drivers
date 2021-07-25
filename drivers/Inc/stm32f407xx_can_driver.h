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



/* CAN_InitStatus*/
#define CAN_InitStatus_Failed              0 /*!< CAN initialization failed */
#define CAN_InitStatus_Success             1 /*!< CAN initialization OK */

void CAN_PeriperalClockControl(CAN_RegDef_t *pCANx, uint8_t ENorDIS);
uint8_t CANInit (CAN_Handle_t *pCANHanlde);
void CAN_DeInit (CAN_RegDef_t *CANx);

#endif /* INC_STM32F407XX_CAN_DRIVER_H_ */
