/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jul 15, 2021
 *      Author: Quy Long
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_
/*
 * Configuration structure for SPIx peripheral
 */

typedef struct
{
	uint8_t	SPI_DeviceMode;
	uint8_t	SPI_BusConfig;
	uint8_t	SPI_SclkSpeed;
	uint8_t	SPI_DFF;
	uint8_t	SPI_CPOL;
	uint8_t	SPI_CPHA;
	uint8_t	SPI_SSM;		/*!<Software slave management. When the SSM bit is set, the NSS pin input is replaced with the value from the SSI bit.>*/
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */

typedef struct
{
	SPI_RegDef_t	*pSPIx; 	/*!< This holds the based address of SPIx(x:1,2,3) peripheral >*/
	SPI_Config_t	SPIConfig;
	uint8_t			*pTxBuffer;	/*!< This stores the app. Tx buffer address >*/
	uint8_t			*pRxBuffer;	/*!< This stores the app. Rx buffer address >*/
	uint32_t		TxLen;		/*!< To store Tx Length >*/
	uint32_t		RxLen;		/*!< To store Rx Length >*/
	uint8_t			TxState;	/*!< To store Tx State >*/
	uint8_t			RxState;	/*!< To store Rx State >*/

}SPI_Handle_t;

/*
 * SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0

/*
 * SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

/*
 * SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2		0
#define SPI_SCLK_SPEED_DIV4		1
#define SPI_SCLK_SPEED_DIV8		2
#define SPI_SCLK_SPEED_DIV16	3
#define SPI_SCLK_SPEED_DIV32	4
#define SPI_SCLK_SPEED_DIV64	5
#define SPI_SCLK_SPEED_DIV128	6
#define SPI_SCLK_SPEED_DIV256	7

/*
 * SPI_DFF
 */
#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1

/*
 * SPI_CPOL (Clock polarity)
 */
#define SPI_CPOL_LOW		0
#define SPI_CPOL_HIGH		1
/*
 * SPI_CPHA (Clock phase)
 */
#define SPI_CPHA_LOW		0
#define SPI_CPHA_HIGH		1

/*
 * SPI_SSM (Software slave management)
 */
#define SPI_SSM_DISABLE		0
#define SPI_SSM_ENABLE		1

/*
 * SPI related status flags definition
 */
#define SPI_TXE_FLAG	( 1 << SPI_SR_TXE ) 	//Tx buffer empty flag (TXE)
#define SPI_RXNE_FLAG	( 1 << SPI_SR_RXNE ) 	//Rx buffer not empty (RXNE)
#define SPI_BUSY_FLAG	( 1 << SPI_SR_TBSY )	//Busy flag: SPI is busy in communication or Tx buffer is not empty

/*
 * SPI Application state
 */
#define SPI_READY		0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX	2

/*
 * SPI Application Events
 */
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRC_ERR	4

/****************************************************************************************************
 * 								APIs supported by this driver
 *			 For more information about APIs check the function definitions
 *
 ****************************************************************************************************/
/*
 * Peripheral Clock Setup
 */
void SPI_PeriperalClockControl(SPI_RegDef_t *pSPIx, uint8_t ENorDIS);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHanlde);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHanlde, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHanlde, uint8_t *pRxBuffer, uint32_t Len);

/*
 *	SPI_IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDIS);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
/*
 * Other peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDIS);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDIS);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDIS);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
/*
 *
 */

void SPI_AppEventCallback(SPI_Handle_t *pSPIHanlde, uint8_t AppEvent);
#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
