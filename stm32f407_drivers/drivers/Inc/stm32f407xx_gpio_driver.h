//this header file is a driver specific header
#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h" //MCU specific data,
#include<stdint.h>

//this is a configureation structure  for a gpio pin
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_pinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;





//handler structure for a GPIO pin
typedef struct
{
	//pointer to hold the base address for the GPIO peripheral
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handler_t;


#define FALLEDGE		4
#define RISEEDGE		5
#define RFT				6


#define PUSHPULL		0
#define OPENDRAIN		1


#define LOW				0
#define MEDIUM			1
#define FAST			2
#define SPEEDHIGH			3

#define NOPUPD			0
#define PULLUP			1
#define PULLDOWN		2

#define AF0				0
#define AF1				1
#define AF2				2
#define AF3				3
#define AF4				4
#define AF5				5
#define AF6				6
#define AF7				7
#define AF8				8
#define AF9				9
#define AF10			10
#define AF11			11
#define AF12			12
#define AF13			13
#define AF14			14
#define AF15			15


//prototypes for API supported by this driver


/*******
 *
 * @fn				-GPIO_PLCK_Control
 *
 * @breif			-This function enables or disables the peripheral clock for the given gpio port
 *
 * @param			-base address of the gpio peripheral
 * @param			-enable or disable the clock
 * @param			-
 *
 * @return			-none
 *
 * @note			-none
 */
void GPIO_PLCK_Control(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_DI();
			}
			else if(pGPIOx == GPIOB)
			{
				GPIOB_PCLK_DI();
			}
			else if(pGPIOx == GPIOC)
			{
				GPIOC_PCLK_DI();
			}
			else if(pGPIOx == GPIOD)
			{
				GPIOD_PCLK_DI();
			}
			else if(pGPIOx == GPIOE)
			{
				GPIOE_PCLK_DI();
			}
			else if(pGPIOx == GPIOF)
			{
				GPIOF_PCLK_DI();
			}
			else if(pGPIOx == GPIOG)
			{
				GPIOG_PCLK_DI();
			}
			else if(pGPIOx == GPIOH)
			{
				GPIOH_PCLK_DI();
			}
			else if(pGPIOx == GPIOI)
			{
				GPIOI_PCLK_DI();
			}
	}
}


/*******
 *
 * @fn				-GPIO_Init
 *
 * @breif			-This function configures the all the SETTINGS peripheral registers for the GPIO pin
 *
 * @param			-base address of the GPIO port
 * @param			-
 * @param			-
 *
 * @return			-none
 *
 * @note			-Wrote this code differently than the instructor, also made some corrections to his code
 */
void GPIO_Init(GPIO_Handler_t *pGPIOHandle)
{
	 uint32_t temp=0; //temp. register

		 //enable the peripheral clock

	 	 GPIO_PLCK_Control(pGPIOHandle->pGPIOx, ENABLE);

		//1 . configure the mode of gpio pin

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= ANALOG)
		{
			//the non interrupt mode
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
			pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
			pGPIOHandle->pGPIOx->MODER |= temp; //setting

		}else
		{
			//this part will code later . ( interrupt mode)
			if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==FALLEDGE )
			{
				//1. configure the FTSR
				EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				//Clear the corresponding RTSR bit
				EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==RISEEDGE )
			{
				//1 . configure the RTSR
				EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				//Clear the corresponding RTSR bit
				EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == RFT )
			{
				//1. configure both FTSR and RTSR
				EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				//Clear the corresponding RTSR bit
				EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			}

			//2. configure the GPIO port selection in SYSCFG_EXTICR
			uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
			uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
			uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);

			//3 . enable the exti interrupt delivery using IMR
			EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
		}

		//2. configure the speed
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;

		//3. configure the pupd settings
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_pinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->PUPDR |= temp;


		//4. configure the optype
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->OTYPER |= temp;

		//5. configure the alt functionality
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == ALTFUNC)
		{
			//configure the alt function registers.
			uint8_t temp1, temp2;

			temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
			temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
			pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
			pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
		}
}


/*******
 *
 * @fn				-GPIO_GPIO_DeInit
 *
 * @breif			-This function resets the peripheral
 *
 * @param			-base address of the gpio peripheral
 * @param			-
 * @param			-
 *
 * @return			-none
 *
 * @note			-none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}
			else if(pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}
			else if(pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}
			else if(pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET();
			}
			else if(pGPIOx == GPIOE)
			{
				GPIOE_REG_RESET();
			}
			else if(pGPIOx == GPIOF)
			{
				GPIOF_REG_RESET();
			}
			else if(pGPIOx == GPIOG)
			{
				GPIOG_REG_RESET();
			}
			else if(pGPIOx == GPIOH)
			{
				GPIOH_REG_RESET();
			}
			else if(pGPIOx == GPIOI)
			{
				GPIOI_REG_RESET();
			}
}


/*******
 *
 * @fn				-GPIO_ReadFromInputPin
 *
 * @breif			-Read the pin number and the corresponding bit register
 *
 * @param			-base address of the gpio peripheral
 * @param			-the pin you want to read from
 * @param			-
 *
 * @return			- 0 or 1
 *
 * @note			-none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & (0x00000001));
	return value;
}

/*******
 *
 * @fn				-GPIO_ReadFromInputPort
 *
 * @breif			-Read all the inputs on a port
 *
 * @param			-base address of the gpio peripheral
 * @param			-
 * @param			-
 *
 * @return			- 16 bit binary
 *
 * @note			-none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)(pGPIOx->IDR);
	return value;
}


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint16_t Value)
{
	if(Value == SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}


void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}



void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber%32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber%64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber%32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber%64));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//fin the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8- NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear pending reggister bit
		EXTI->PR |= (1 << PinNumber);
	}
}

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
