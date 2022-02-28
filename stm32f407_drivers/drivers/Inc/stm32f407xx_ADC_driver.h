/*
 * stm32f407xx_ADC_driver.h
 *
 *  Created on: Feb 5, 2022
 *      Author: mmazzi
 */

#ifndef INC_STM32F407XX_ADC_DRIVER_H_
#define INC_STM32F407XX_ADC_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint32_t ADCChannel;
	uint32_t ADCConversion;
	uint8_t ADCResolution;
	uint8_t ADCCycles;

}ADC_Config_t;

//Resolution
#define BIT12		0
#define BIT10		1
#define BIT8		2
#define BIT6		3

//Cycles
#define C3			0
#define C15			1
#define C28			2
#define C56			3
#define C84			4
#define C112		5
#define C144		6
#define C480		7

#define SINGLE		0
#define CONT		1

typedef struct
{
	//pointer to hold the base address for the GPIO peripheral
	ADC_RegDef_t *pADCx;
	ADC_Config_t ADC_Config;

}ADC_Handler_t;

void delay2(void);

void ADC_Clk_EnorDi(ADC_RegDef_t *pADCx, uint8_t EnorDi);

void ADC_Init(ADC_RegDef_t *pADCHandle);

void ADC_Init_LR(ADC_RegDef_t *pADCHandle);
void ADC_Init_UD(ADC_RegDef_t *pADCHandle);
void ADC_Wrist(ADC_RegDef_t *pADCHandle);

uint32_t EdgeCondition(uint32_t val);

uint8_t LR_Inc_Flg;
uint8_t UD_Inc_Flg;
uint8_t LR_Dec_Flg;
uint8_t UD_Dec_Flg;
uint8_t Wrist_Inc_Flg;
uint8_t Wrist_Dec_Flg;

#endif /* INC_STM32F407XX_ADC_DRIVER_H_ */
