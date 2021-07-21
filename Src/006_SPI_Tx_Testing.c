/*
 * 06_SPI_Tx_Testing.c
 *
 *  Created on: Jul 15, 2021
 *      Author: Quy Long
 */
/*	PB14 --> SPI2_MISO
 *	PB15 --> SPI2_MOSI
 *	PB13 --> SPI2_SCLK
 *	PB12 --> SPI2_NSS
 *
 * 	AFT function mode : 5
 */
#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++); //Temp Delay
}
void SPI_GPIOInit(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType =GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12; // PB12 --> SPI2_NSS
	GPIO_Init(&SPIPins);
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13; // PB13 --> SPI2_SCLK
	GPIO_Init(&SPIPins);
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14; // PB14 --> SPI2_MISO
	GPIO_Init(&SPIPins);
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15; // PB15 --> SPI2_MOSI
	GPIO_Init(&SPIPins);

}

void SPI2_Init(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; // Generates SCLK of 8MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_ENABLE; // Software slave management enabled for NSS pin

	SPI_Init(&SPI2Handle);

}
int main(void)
{
	char user_data[] = "Hello world";

	// This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI_GPIOInit();

	// This function is used to initialize the SPI2 peripheral parameters
	SPI2_Init();

	// This function makes NSS signal internally high and avoids MODF error
	SPI_SSIConfig(SPI2, ENABLE);

	// Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);


	// Send Data
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	// Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1)
		{
		// Enable the SPI2 peripheral
			SPI_PeripheralControl(SPI2, ENABLE);

			// Send Data
			SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

			// Disable the SPI2 peripheral
			SPI_PeripheralControl(SPI2, DISABLE);
			delay();
		}

	return 0;
}
