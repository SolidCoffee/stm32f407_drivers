/*
 * DualThumbstick.c
 *
 *  Created on: Feb 25, 2022
 *      Author: mmazzi
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_timer.h"
#include "stm32f407xx_ADC_driver.h"
#include<stdio.h>
#include<string.h>

void delay(void)
{
	for(uint32_t i = 0 ; i < 100000; i ++);
}

void PWM_GPIOInits(void)
{
	GPIO_Handler_t PWMPins;

	PWMPins.pGPIOx = GPIOB;

	PWMPins.GPIO_PinConfig.GPIO_PinMode = ALTFUNC;
	PWMPins.GPIO_PinConfig.GPIO_PinAltFunMode = AF2;
	PWMPins.GPIO_PinConfig.GPIO_PinOPType = PUSHPULL;
	PWMPins.GPIO_PinConfig.GPIO_pinPuPdControl = NOPUPD;
	PWMPins.GPIO_PinConfig.GPIO_PinSpeed = SPEEDHIGH;  //might need to look into this

	//PWM Pin config
	PWMPins.GPIO_PinConfig.GPIO_PinNumber = 6;
	GPIO_Init(&PWMPins);

	PWMPins.GPIO_PinConfig.GPIO_PinNumber = 7;
	GPIO_Init(&PWMPins);

	PWMPins.GPIO_PinConfig.GPIO_PinNumber = 8;
	GPIO_Init(&PWMPins);

	PWMPins.GPIO_PinConfig.GPIO_PinNumber = 9;
	GPIO_Init(&PWMPins);
}



int main(void)
{
	PWM_GPIOInits();

	while(1)
	{

	}

}
