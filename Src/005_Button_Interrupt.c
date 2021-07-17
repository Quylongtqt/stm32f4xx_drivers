/*
 * 005_Button_Interrupt.c
 *
 *  Created on: Jul 10, 2021
 *      Author: Quy Long
 */
#include <string.h>
#include "stm32f407xx.h"
void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++); //Temp Delay
}

int main(void)
{
	GPIO_Handle_t GpioLED, GpioBtn;

	memset(&GpioLED, 0, sizeof(GpioLED));
	memset(&GpioBtn, 0, sizeof(GpioBtn));

	// Configuration for LED
	GpioLED.pGPIOx = GPIOD;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriperalClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLED);

	// Configuration for Button
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriperalClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);

	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRIO15);
	while(1);

	return 0;
}

void EXTI0_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
