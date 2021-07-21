/*
 * 009_SPI_Message_Rcv_IT.c
 *
 *  Created on: Jul 18, 2021
 *      Author: Quy Long
 */
/*
 * Exercise:
 *
 * STM32F4 Discovery board (Master) receives a message from the Arduino board (Slave) over SPI
 *
 * 1. User enters the message using Arduino serial monitor Terminal
 * 2. Arduino board notifies the STM32 board about the message availability
 * 3. STM32 device reads and prints the message
 *
 */

/*	PB14 --> SPI2_MISO
 *	PB15 --> SPI2_MOSI
 *	PB13 --> SPI2_SCLK
 *	PB12 --> SPI2_NSS
 * 	AFT function mode : 5
 *
 *	PD06 --> High2Low Interrupt from Slave
 *
 */
/*
 * Note: Follow the instructions to test this code
 * 1. Download this code to STM32 board, acts as Master
 * 2. Download Slave code(003SPISlaveUartReadOverSPI.ino) into Arduino board
 * 3. Reset both the boards
 * 4. Enable SWV ITM data console to see the message
 * 4. Open Arduino IDE serial monitor tool
 * 6. Type something and send the message
 */
#include <string.h>
#include <stdio.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#define MAX_LEN	500
char RcvBff[MAX_LEN];
volatile char ReadByte;
volatile uint8_t rcvStop = 0;
volatile uint8_t dataAvailable = 0;

SPI_Handle_t SPI2Handle;

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++); //Temp Delay
}
void SPI2_GPIOInit(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType =GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// PB12 --> SPI2_NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

	// PB13 --> SPI2_SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// PB14 --> SPI2_MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	// PB15 --> SPI2_MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

}

void SPI2_Init(void)
{
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32; // Generates SCLK of 2MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_ENABLE; // Software slave management enabled for NSS pin

	SPI_Init(&SPI2Handle);

}

/*
 * This function configures the GPIO Pin over which SPI peripheral issues data available interrupt
 */
void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t spiIntPin;
	memset(&spiIntPin, 0, sizeof(spiIntPin));

	spiIntPin.pGPIOx = GPIOD;
	spiIntPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	spiIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	spiIntPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	spiIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&spiIntPin);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

}

int main(void)
{
	uint8_t	dummy = 0xff;

	Slave_GPIO_InterruptPinInit();
	SPI2_GPIOInit();
	SPI2_Init();

	/*
	 * Making SSOE 1 does NSS output enable.
	 * THe NSS pin is automatically managed by the HW
	 * i.e when SPE = 1, NSS will be pulled to low
	 * and NSS pin will be high when SPE = 0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);
	SPI_IRQInterruptConfig(IRQ_NO_SPI2, ENABLE);

	while(1)
	{
		rcvStop = 0;

		while(!dataAvailable); // Wait till data available interrupt from transmitter device (Slave)

		GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, DISABLE);

		// Enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		while(!rcvStop)
		{
			/*
			 * Fetch the data from the SPI peripheral byte by byte in interrupt mode
			 */
			while(SPI_SendDataIT(&SPI2Handle, &dummy, 1) == SPI_BUSY_IN_TX);
			while(SPI_ReceiveDataIT(&SPI2Handle, &ReadByte, 1) == SPI_BUSY_IN_RX);
		}

		// Confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		// Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		printf("Rcv data = %s\n", RcvBff);

		dataAvailable = 0;

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	}

	return 0;
}

/*
 *	Run when a data byte is received from the peripheral oer SPI
 */
void SPI2_IRQHandler(void)
{
	SPI_IRQHandling(&SPI2Handle);
}

void SPI_AppEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	static uint32_t i =0;
	/*
	 * In the Rx complete event, copy data into Rcv Buffer.
	 */
	if(AppEvent == SPI_EVENT_RX_CMPLT)
	{
		RcvBff[i++] = ReadByte;
		if(ReadByte == '\0' || (i == MAX_LEN))
		{
			rcvStop = 1;
			RcvBff[i-1] = '\0';
			i = 0;
		}
	}
}

/*
 * Slave data available interrupt handler
 */
void EXT9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	dataAvailable = 1;
}

