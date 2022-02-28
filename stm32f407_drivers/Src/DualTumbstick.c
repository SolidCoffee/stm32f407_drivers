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

uint32_t x=10;
uint32_t y=10;
uint32_t j=10;
uint32_t checker=0;

void ServoHandle(void)
{

	if(UD_Inc_Flg)
	{
		x++;
		x=EdgeCondition(x);
		ServoAngle(TIM4, x, MID);
	}
	else if(UD_Dec_Flg)
	{
		x--;
		x=EdgeCondition(x);
		ServoAngle(TIM4, x, MID);
	}
	UD_Inc_Flg=0; UD_Dec_Flg=0;

	if(LR_Inc_Flg)
	{
		y++;
		y=EdgeCondition(y);
		ServoAngle(TIM4, y, BOTTOM);
	}
	else if(LR_Dec_Flg)
	{
		y--;
		y=EdgeCondition(y);
		ServoAngle(TIM4, y, BOTTOM);
	}
	LR_Inc_Flg=0; LR_Dec_Flg=0;

	if(Wrist_Inc_Flg)
	{
		j++;
		j=EdgeCondition(j);
		ServoAngle(TIM4, j,TOP);
	}
	else if(Wrist_Dec_Flg)
	{
		j--;
		j=EdgeCondition(j);
		ServoAngle(TIM4, j,TOP);
	}
	Wrist_Inc_Flg=0; Wrist_Dec_Flg=0;
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
	GPIO_Init(&ADCPins);

	//ADCPins.GPIO_PinConfig.GPIO_PinNumber = 7;
	//GPIO_Init(&ADCPins);
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
}

void PWMTIM(void)
{
	TIM_Handler_t TIMPWM;

	TIMPWM.pTIMx = TIM4;
	TIMPWM.TIM_Config.TIM_Direction = UPCNT;
	TIMPWM.TIM_Config.TIM_ARR = 479;
	TIMPWM.TIM_Config.TIM_Prescaler = 99;
	TIMPWM.TIM_Config.TIM_Mode = EDGE;
	TIMPWM.TIM_Config.PWM_Mode = PWM1;
	TIMPWM.TIM_Config.TIM_RestPostion = DISABLE;
	TIMPWM.TIM_Config.TIM_Polarity = POLHIGH;
	TIMPWM.TIM_Config.PWM_Channel = 1;

	PWM2_5_Init(&TIMPWM);

	TIMPWM.TIM_Config.PWM_Channel = 2;

	PWM2_5_Init(&TIMPWM);

	TIMPWM.TIM_Config.PWM_Channel = 3;

	PWM2_5_Init(&TIMPWM);
}


int main(void)
{
	GPIOInits();
	ADC_Clk_EnorDi(ADC1, ENABLE);
	PWM_GPIOInits();
	TIM2_5_CLKEnable(TIM4, ENABLE); //Enables HSI and TIM3 peripheral clock
	PWMTIM();

	while(1)
	{
		ADC_Init_LR(ADC1);

		ADC_Init_UD(ADC1);

		ADC_Wrist(ADC1);

		ServoHandle();
	}

}
