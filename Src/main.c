/*
 * main.c
 *
 *  Created on: Jul 10, 2021
 *      Author: Quy Long
 */
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

int main(void)
{
	return 0;
}

void EXTI0_IRQHandle(void)
{
	// Handle the interrupt
	GPIO_IRQHandling(0);
}
