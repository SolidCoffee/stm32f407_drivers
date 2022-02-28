#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_timer.h"
#include "stm32f407xx_ADC_driver.h"
#include<stdio.h>
#include<string.h>


uint32_t i=0;

void ADC_Init_LR(ADC_RegDef_t *pADCHandle)
{
	//Enable ADC
	pADCHandle->CR2 |= (1 << 0);

	//continous conversion setting
	pADCHandle->CR2 |= (1 << 1);

	//clearing channel
	pADCHandle->SQR3 &= ~(31 << 0);

	//Selecting channel
	pADCHandle->SQR3 |= (1 << 0);

	//cycle edit
	pADCHandle->SMPR2 |= (0 << 4);

	//resolution
	pADCHandle->CR1 |= (1 << 24);

	//start conversion of regular channels
	pADCHandle->CR2 |= (1 << 30);

	if(pADCHandle->DR > 600)
	{
		GPIO_WriteToOutputPin(GPIOA, 5, SET);  //blue
	}
	else if(pADCHandle->DR < 500)
	{
		GPIO_WriteToOutputPin(GPIOA, 5, 0);
	}
	//Disable ADC
	pADCHandle->CR2 &= ~(1 << 0);
}

void ADC_Init_UD(ADC_RegDef_t *pADCHandle)
{
	//Enable ADC
	pADCHandle->CR2 |= (1 << 0);

	//continous conversion setting
	pADCHandle->CR2 |= (1 << 1);

	//clearing channel
	pADCHandle->SQR3 &= ~(31 << 0);

	//Selecting channel
	pADCHandle->SQR3 |= (3 << 0);

	//cycle edit
	pADCHandle->SMPR2 |= (0 << 4);

	//resolution
	pADCHandle->CR1 |= (1 << 24);

	//start conversion of regular channels
	pADCHandle->CR2 |= (1 << 30);

	if(pADCHandle->DR > 600)
	{
		GPIO_WriteToOutputPin(GPIOA, 7, SET);  //red
	}
	else if(pADCHandle->DR < 500)
	{
		GPIO_WriteToOutputPin(GPIOA, 7, 0);
	}
	//Disable ADC
	pADCHandle->CR2 &= ~(1 << 0);
}
void GPIOInits(void)
{
	GPIO_Handler_t ADCPins;

	ADCPins.pGPIOx = GPIOA;

	ADCPins.GPIO_PinConfig.GPIO_PinMode = ANALOG;
	ADCPins.GPIO_PinConfig.GPIO_PinAltFunMode = AF0;
	ADCPins.GPIO_PinConfig.GPIO_PinOPType = PUSHPULL;
	ADCPins.GPIO_PinConfig.GPIO_pinPuPdControl = NOPUPD;
	ADCPins.GPIO_PinConfig.GPIO_PinSpeed = SPEEDHIGH;

	//PWM Pin config
	ADCPins.GPIO_PinConfig.GPIO_PinNumber = 1;
	GPIO_Init(&ADCPins);

	ADCPins.GPIO_PinConfig.GPIO_PinNumber = 3;
	GPIO_Init(&ADCPins);

	ADCPins.GPIO_PinConfig.GPIO_PinNumber = 5;
	ADCPins.GPIO_PinConfig.GPIO_PinMode = OUTPUT;
	GPIO_Init(&ADCPins);

	ADCPins.GPIO_PinConfig.GPIO_PinNumber = 7;
	ADCPins.GPIO_PinConfig.GPIO_PinMode = OUTPUT;
	GPIO_Init(&ADCPins);
}


int main(void)
{

	GPIOInits();
	ADC_Clk_EnorDi(ADC1, ENABLE);

	while(1)
	{
		ADC_Init_LR(ADC1);  //left right


		ADC_Init_UD(ADC1);  //up down
	}
}
