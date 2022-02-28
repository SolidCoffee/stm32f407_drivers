#include "stm32f407xx_timer.h"

void TIM10_11_13_14_CLKEnable(TIM10_11_13_14_RegDef_t *pTIMx, uint8_t EnorDi)
{
	RCC->AHB1ENR |= (1 << 0); // setting HSI on
	if(EnorDi == ENABLE)
	{
		if(pTIMx == TIM10)
		{
			TIM10_CLK_EN(); //Enabling the TIM3 peripheral clock
		}
		else if(pTIMx == TIM11)
		{
			TIM11_CLK_EN();
		}
		else if(pTIMx == TIM13)
		{
			TIM13_CLK_EN();
		}
		else if(pTIMx == TIM14)
		{
			TIM14_CLK_EN();
		}
	}
	else
	{
		if(pTIMx == TIM10)
		{
			TIM10_CLK_DI(); //Disabling the TIM3 peripheral clock
		}
		else if(pTIMx == TIM11)
		{
			TIM11_CLK_DI();
		}
		else if(pTIMx == TIM13)
		{
			TIM13_CLK_DI();
		}
		else if(pTIMx == TIM14)
		{
			TIM14_CLK_DI();
		}
	}
}

void TIM2_5_CLKEnable(TIM2_5_RegDef_t *pTIMx, uint8_t EnorDi)
{
	RCC->AHB1ENR |= (1 << 0); // setting HSI on
	if(EnorDi == ENABLE)
	{
		if(pTIMx == TIM3)
		{
			TIM3_CLK_EN(); //Enabling the TIM3 peripheral clock
		}
		else if(pTIMx == TIM4)
		{
			TIM4_CLK_EN();
		}
		else if(pTIMx == TIM2)
		{
			TIM2_CLK_EN();
		}
		else if(pTIMx == TIM5)
		{
			TIM5_CLK_EN();
		}
	}
	else
	{
		if(pTIMx == TIM3)
		{
			TIM3_CLK_DI(); //Disabling the TIM3 peripheral clock
		}
		else if(pTIMx == TIM4)
		{
			TIM4_CLK_DI();
		}
		else if(pTIMx == TIM2)
		{
			TIM2_CLK_DI();
		}
		else if(pTIMx == TIM5)
		{
			TIM5_CLK_DI();
		}
	}

}

void PWM2_5_Init(TIM_Handler_t *pTIMHandle)
{
	//CR1 peripheral setting

	pTIMHandle->pTIMx->CR1 |= (0 << 3); //clears the one pulse mode bit
	pTIMHandle->pTIMx->CR1 |= (pTIMHandle->TIM_Config.TIM_Mode << 5); //Sets timer mode
	pTIMHandle->pTIMx->CR1 |= (pTIMHandle->TIM_Config.TIM_Direction << 4); //sets direction
	pTIMHandle->pTIMx->CR1 |= (1 << 7); // ARR register is buffered

	//CCRM1
	if(pTIMHandle->TIM_Config.PWM_Channel == 1)
	{
		pTIMHandle->pTIMx->CCMR1 |= (pTIMHandle->TIM_Config.PWM_Mode << 4);  //pwm mode config
		pTIMHandle->pTIMx->CCMR1 |= (1 << 3); //Preload register on TIMx_CCR1 enabled.
	}
	else if(pTIMHandle->TIM_Config.PWM_Channel == 2)
	{
		pTIMHandle->pTIMx->CCMR1 |= (pTIMHandle->TIM_Config.PWM_Mode << 12);  //pwm mode config
		pTIMHandle->pTIMx->CCMR1 |= (1 << 11); //Preload register on TIMx_CCR1 enabled.
	}
	else if(pTIMHandle->TIM_Config.PWM_Channel == 3)
	{
		pTIMHandle->pTIMx->CCMR2 |= (pTIMHandle->TIM_Config.PWM_Mode << 4);  //pwm mode config
		pTIMHandle->pTIMx->CCMR2 |= (1 << 3); //Preload register on TIMx_CCR1 enabled.
	}
	else if(pTIMHandle->TIM_Config.PWM_Channel == 4)
	{
		pTIMHandle->pTIMx->CCMR2 |= (pTIMHandle->TIM_Config.PWM_Mode << 12);  //pwm mode config
		pTIMHandle->pTIMx->CCMR2 |= (1 << 11); //Preload register on TIMx_CCR1 enabled.
	}

	//CCER
	if(pTIMHandle->TIM_Config.PWM_Channel == 1)
	{
		pTIMHandle->pTIMx->CCER |= (pTIMHandle->TIM_Config.TIM_Polarity << 1);
		pTIMHandle->pTIMx->CCER |= (1 << 0); //output enable
	}
	else if(pTIMHandle->TIM_Config.PWM_Channel == 2)
	{
		pTIMHandle->pTIMx->CCER |= (pTIMHandle->TIM_Config.TIM_Polarity << 5);
		pTIMHandle->pTIMx->CCER |= (1 << 4); //output enable
	}
	else if(pTIMHandle->TIM_Config.PWM_Channel == 3)
	{
		pTIMHandle->pTIMx->CCER |= (pTIMHandle->TIM_Config.TIM_Polarity << 9);
		pTIMHandle->pTIMx->CCER |= (1 << 8); //output enable
	}
	else if(pTIMHandle->TIM_Config.PWM_Channel == 4)
	{
		pTIMHandle->pTIMx->CCER |= (pTIMHandle->TIM_Config.TIM_Polarity << 13);
		pTIMHandle->pTIMx->CCER |= (1 << 12); //output enable
	}

	//Prescelar setting and ARR
	pTIMHandle->pTIMx->PSC = pTIMHandle->TIM_Config.TIM_Prescaler;
	pTIMHandle->pTIMx->ARR = pTIMHandle->TIM_Config.TIM_ARR;


	//Rest postion of motor
	if(pTIMHandle->TIM_Config.TIM_RestPostion > 0)
	{
		if(pTIMHandle->TIM_Config.PWM_Channel == 1)
		{
			pTIMHandle->pTIMx->CCR1 = pTIMHandle->TIM_Config.TIM_RestPostion;
		}
		else if(pTIMHandle->TIM_Config.PWM_Channel == 2)
		{
			pTIMHandle->pTIMx->CCR2 = pTIMHandle->TIM_Config.TIM_RestPostion;
		}
		else if(pTIMHandle->TIM_Config.PWM_Channel == 3)
		{
			pTIMHandle->pTIMx->CCR3 = pTIMHandle->TIM_Config.TIM_RestPostion;
		}
		else if(pTIMHandle->TIM_Config.PWM_Channel == 4)
		{
			pTIMHandle->pTIMx->CCR4 = pTIMHandle->TIM_Config.TIM_RestPostion;
		}
	}
	pTIMHandle->pTIMx->EGR |= (1 << 0);//enables the UG bit

	pTIMHandle->pTIMx->CR1 |= (1 << 0); //enables the Counter

}

void ServoAngle(TIM2_5_RegDef_t *pTIMx, uint32_t Angle, uint8_t Channel)
{
	uint32_t DutyCycle = (Angle*1.188)+77;
	if(Channel == 1)
	{
		pTIMx->CCR1 = DutyCycle;
	}
	else if(Channel == 2)
	{
		pTIMx->CCR2 = DutyCycle;
	}
	else if(Channel == 3)
	{
		pTIMx->CCR3 = DutyCycle;
	}
	else if(Channel == 4)
	{
		pTIMx->CCR4 = DutyCycle;
	}

}
