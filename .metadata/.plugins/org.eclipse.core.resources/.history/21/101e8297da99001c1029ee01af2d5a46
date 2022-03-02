#ifndef INC_STM32F407XX_TIMER_H_
#define INC_STM32F407XX_TIMER_H_

#include "stm32f407xx.h"


typedef struct
{
	uint32_t TIM_ARR;
	uint16_t TIM_Prescaler;
	uint8_t TIM_Polarity;
	uint32_t TIM_RestPostion;
	uint8_t TIM_Mode;
	uint8_t TIM_Direction;
	uint8_t PWM_Mode;
	uint8_t PWM_Channel;

}TIM2_5_PinConfig_t;

//macros for direction
#define UPCNT		0
#define DOWNCNT		1

//Macros for timer mode
#define EDGE		0
#define CENTER1		1
#define CENTER2		2
#define CENTER3		3

//marcros for PWM mode
#define PWM1		6
#define PWM2		7

//Macros for polarity
#define POLHIGH		0
#define POLLOW		1

//macros for rest position
#define DISABLE		0

//motors
#define BOTTOM			2
#define MID			1
#define TOP			3





//handler structure for a TIMx peri
typedef struct
{
	//pointer to hold the base address for the GPIO peripheral
	TIM2_5_RegDef_t *pTIMx;
	TIM2_5_PinConfig_t TIM_Config;

}TIM_Handler_t;

void TIM10_11_13_14_CLKEnable(TIM10_11_13_14_RegDef_t *pTIMx, uint8_t EnorDi);
void TIM2_5_CLKEnable(TIM2_5_RegDef_t *pTIMx, uint8_t EnorDi);
void PWM2_5_Init(TIM_Handler_t *pTIMHandle);

void ServoAngle(TIM2_5_RegDef_t *pTIMx, uint32_t Angle, uint8_t Channel);


#endif /* INC_STM32F407XX_TIMER_H_ */
