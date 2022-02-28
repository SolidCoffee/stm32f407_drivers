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
	if(Wrist_flg==0)
	{
		if(UD_Inc_Flg)
		{
			x++;
			if(x >= 180)
			{
				x=180;
			}
			else if(x <= 10)
			{
				x=10;
			}
			ServoAngle(TIM4, x, MID);
		}
		else if(UD_Dec_Flg)
		{
			x--;
			if(x >= 180)
			{
				x=180;
			}
			else if(x <= 10)
			{
				x=10;
			}
			ServoAngle(TIM4, x, MID);
		}
		UD_Inc_Flg=0;
		UD_Dec_Flg=0;

		if(LR_Inc_Flg)
		{
			y++;
			if(y >= 180)
			{
				y=180;
			}
			else if(y <= 10)
			{
				y=10;
			}
			ServoAngle(TIM4, y, BOTTOM);
		}
		else if(LR_Dec_Flg)
		{
			y--;
			if(y >= 180)
			{
				y=180;
			}
			else if(y <= 10)
			{
				y=10;
			}
			ServoAngle(TIM4, y, BOTTOM);
		}
		LR_Inc_Flg=0;
		LR_Dec_Flg=0;

	}
	else
	{
		if(UD_Inc_Flg)
		{
			j++;
			if(j >= 180)
			{
				j=180;
			}
			else if(j <= 10)
			{
				j=10;
			}
			ServoAngle(TIM4, j, TOP);
		}
		else if(UD_Dec_Flg)
		{
			j--;
			if(j >= 180)
			{
				j=180;
			}
			else if(j <= 10)
			{
				j=10;
			}
			ServoAngle(TIM4, j, TOP);
		}
		UD_Inc_Flg=0;
		UD_Dec_Flg=0;
	}

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

void GPIO_ButtonInit()
{
	GPIO_Handler_t GPIOBtn;

	GPIOBtn.pGPIOx = GPIOD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = 5;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = FALLEDGE;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = LOW;
	GPIOBtn.GPIO_PinConfig.GPIO_pinPuPdControl =PULLUP;

	GPIO_Init(&GPIOBtn);

	//IRQ config
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,1);
	GPIO_IRQITConfig(IRQ_NO_EXTI9_5,ENABLE);
}


int main(void)
{
	GPIOInits();
	ADC_Clk_EnorDi(ADC1, ENABLE);
	PWM_GPIOInits();
	TIM2_5_CLKEnable(TIM4, ENABLE); //Enables HSI and TIM3 peripheral clock

	GPIO_ButtonInit();

	PWMTIM();

	//delay2();
	while(1)
	{
		ADC_Init_LR(ADC1);

		ADC_Init_UD(ADC1);

		//ADC_Wrist(ADC1);

		ServoHandle();


		//ServoAngle(TIM4, 100, TOP);
		//ServoAngle(TIM4, 100, MID);
		//ServoAngle(TIM4, 80, BOTTOM);
	}


	/*ServoAngle(TIM4, 100, 3);
	ServoAngle(TIM4, 10, 1);
	ServoAngle(TIM4, 180, 2);
	while(1)
		{
			for(uint32_t i = 15; i < 180; i++)
			{
				ServoAngle(TIM4, i, 1);
				ServoAngle(TIM4, i, 2);
				ServoAngle(TIM4, i, 3);
				delay();
			}
			for(uint32_t i = 180; i > 15; i--)
			{
				ServoAngle(TIM4, i, 1);
				ServoAngle(TIM4, i, 2);
				ServoAngle(TIM4, i, 3);
				delay();
			}
		}*/

	/*while(1)
	{
		ServoAngle(TIM4, 1, 1);
		while(! GPIO_ReadFromInputPin(GPIOA,0));
		delay();
		ServoAngle(TIM4, 270, 1);
		while(! GPIO_ReadFromInputPin(GPIOA,0));
		delay();
	}*/



}

void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(5);

	checker++;

	if(checker==3)
	{
		checker=2;
	}

	if(checker==2)
	{
		if(Wrist_flg)
		{
			Wrist_flg =0;
		}
		else
		{
			Wrist_flg=1;
		}
		checker=0;
	}
}

/*while(1)
	{
		for(uint32_t i = 1; i < 270; i++)
		{
			ServoAngle(TIM4, i, 1);
			ServoAngle(TIM4, i, 2);
			delay();
		}
		for(uint32_t i = 270; i > 1; i--)
		{
			ServoAngle(TIM4, i, 1);
			ServoAngle(TIM4, i, 2);
			delay();
		}
	}*/
