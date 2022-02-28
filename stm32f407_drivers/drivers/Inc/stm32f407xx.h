#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>
#include<stddef.h>

#define __vo volatile

/******************************************************************************
Processor specific details
*/

//interrupt set enable registers
#define NVIC_ISER0			((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1			((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2			((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3			((__vo uint32_t*)0xE000E10C)

//Interrupt clear enable register
#define NVIC_ICER0			((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2			((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0xE000E18C)

//priority register interrupts
#define NVIC_PR_BASE_ADDR	((__vo uint32_t*)0xE0004000)

#define NO_PR_BITS_IMPLEMENTED		4



//memory addresses
#define FLASH_BASEADDR		0x08000000U  //THis is the base address of the flash
#define SRAM1_BASEADDR		0x20000000U //this is the base address of the SRAM1
#define SRAM2_BASEADDR		0x20001C00U  // this is the base address of the SRAM2
#define SRAM 				SRAM1_BASEADDR  //this is the base address of the SRAM which is equal to the SRAM1 base address
#define ROM					0x1fff0000U //this is the ROM base address

//bus addresses
#define PERIPH_BASE 		0x40000000U
#define APB1PERIPH_BASE 	PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x50000000U

//ADC Offset
#define ADC_2_OFFSET		0x100U
#define ADC_3_OFFSET		0x200U

//AHB1 peripheral addresses
#define GPIOA_BASE			((AHB1PERIPH_BASE)+(0x00))
#define GPIOB_BASE			((AHB1PERIPH_BASE)+(0x0400))
#define GPIOC_BASE			((AHB1PERIPH_BASE)+(0x0800))
#define GPIOD_BASE			((AHB1PERIPH_BASE)+(0x0C00))
#define GPIOE_BASE			((AHB1PERIPH_BASE)+(0x1000))
#define GPIOF_BASE			((AHB1PERIPH_BASE)+(0x1400))
#define GPIOG_BASE			((AHB1PERIPH_BASE)+(0x1800))
#define GPIOH_BASE			((AHB1PERIPH_BASE)+(0x1C00))
#define GPIOI_BASE			((AHB1PERIPH_BASE)+(0x2000))
#define GPIOJ_BASE			((AHB1PERIPH_BASE)+(0x2400))
#define GPIOK_BASE			((AHB1PERIPH_BASE)+(0x2800))
#define CRC_BASE			((AHB1PERIPH_BASE)+(0x3000))
#define RCC_BASE			((AHB1PERIPH_BASE)+(0x3800))
#define FLASH_INTERFACE_REG_BASE			((AHB1PERIPH_BASE)+(0x3C00))
#define BKPSRAM_BASE		((AHB1PERIPH_BASE)+(0x4000))
#define DMA1_BASE			((AHB1PERIPH_BASE)+(0x6000))
#define DMA2_BASE			((AHB1PERIPH_BASE)+(0x6400))
#define ETHERNET_MAC_BASE	((AHB1PERIPH_BASE)+(0x8000))
#define DMA2D_BASE			((AHB1PERIPH_BASE)+(0xB000))
#define USB_OTG_HS_BASE		0x40040000


//APB1 peripheral addresses
#define DAC_BASE			((APB1PERIPH_BASE)+(0x7400))
#define I2C1_BASE			((APB1PERIPH_BASE)+(0x5400))
#define I2C2_BASE			((APB1PERIPH_BASE)+(0x5800))
#define I2C3_BASE			((APB1PERIPH_BASE)+(0x5C00))
#define SPI2_BASE			((APB1PERIPH_BASE)+(0x3800))
#define SPI3_BASE			((APB1PERIPH_BASE)+(0x3C00))
#define USART2_BASE			((APB1PERIPH_BASE)+(0x4400))
#define USART3_BASE			((APB1PERIPH_BASE)+(0x4800))
#define UART4_BASE			((APB1PERIPH_BASE)+(0x4C00))
#define UART5_BASE			((APB1PERIPH_BASE)+(0x5000))
#define TIM2_BASE			((APB1PERIPH_BASE)+(0x0000))
#define TIM3_BASE			((APB1PERIPH_BASE)+(0x0400))
#define TIM4_BASE			((APB1PERIPH_BASE)+(0x0800))
#define TIM5_BASE			((APB1PERIPH_BASE)+(0x0C00))
#define TIM6_BASE			((APB1PERIPH_BASE)+(0x1000))
#define TIM7_BASE			((APB1PERIPH_BASE)+(0x1400))
#define TIM12_BASE			((APB1PERIPH_BASE)+(0x1800))
#define TIM13_BASE			((APB1PERIPH_BASE)+(0x1C00))
#define TIM14_BASE			((APB1PERIPH_BASE)+(0x2000))


//APB2 peripheral addresses
#define SPI1_BASE			((APB2PERIPH_BASE)+(0X3000))
#define USART1_BASE			((APB2PERIPH_BASE)+(0X1000))
#define USART6_BASE			((APB2PERIPH_BASE)+(0X1400))
#define EXTI_BASE			((APB2PERIPH_BASE)+(0X3C00))
#define SYSCFG_BASE			((APB2PERIPH_BASE)+(0X3800))
#define TIM1_BASE			((APB2PERIPH_BASE)+(0X0000))
#define TIM8_BASE			((APB2PERIPH_BASE)+(0X0400))
#define TIM9_BASE			((APB2PERIPH_BASE)+(0X4000))
#define TIM10_BASE			((APB2PERIPH_BASE)+(0X4400))
#define TIM11_BASE			((APB2PERIPH_BASE)+(0X4800))
#define ADC1_BASE			((APB2PERIPH_BASE)+(0x2000))
#define ADC2_BASE			((ADC1_BASE)+ADC_2_OFFSET)
#define ADC3_BASE			((ADC1_BASE)+ADC_3_OFFSET)

typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];  //AFR[0] is "GPIO alternate function low register, AFR[1] is "GPIO alternate function high register

}GPIO_RegDef_t;

#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASE)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASE)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASE)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASE)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASE)
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASE)
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASE)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASE)
#define GPIOI				((GPIO_RegDef_t*)GPIOI_BASE)


typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t Reserved1;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t Reserved2;
	__vo uint32_t Reserved3;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t Reserved4;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t Reserved5;
	__vo uint32_t Reserved6;
	__vo uint32_t AHB1LP_ENR;
	__vo uint32_t AHB2LP_ENR;
	__vo uint32_t AHB3LP_ENR;
	__vo uint32_t Reserved7;
	__vo uint32_t APB1LP_ENR;
	__vo uint32_t APB2LP_ENR;
	__vo uint32_t Reserved8;
	__vo uint32_t Reserved9;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t Reserved10;
	__vo uint32_t Reserved11;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SC_FGR;
}RCC_RegDef_t;

#define RCC					((RCC_RegDef_t*)RCC_BASE)

typedef struct
{
	__vo uint32_t MEMRWP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t RESERVED[2];
	__vo uint32_t CMPCR;
	__vo uint32_t RESERVED2[2];
	__vo uint32_t CFGR;
}SYSCFG_RegDef_t;

#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASE)

//Peripheral register definition structure for EXTI
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t FTSR;
	__vo uint32_t RTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;


#define EXTI				((EXTI_RegDef_t*)EXTI_BASE)

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;

}SPI_RegDef_t;

#define SPI1					((SPI_RegDef_t*)SPI1_BASE)
#define SPI2					((SPI_RegDef_t*)SPI2_BASE)
#define SPI3					((SPI_RegDef_t*)SPI3_BASE)

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;

}I2C_RegDef_t;

#define I2C1					((I2C_RegDef_t*)I2C1_BASE)
#define I2C2					((I2C_RegDef_t*)I2C2_BASE)
#define I2C3					((I2C_RegDef_t*)I2C3_BASE)

typedef struct
{
	uint32_t SR;
	uint32_t DR;
	uint32_t BRR;
	uint32_t CR1;
	uint32_t CR2;
	uint32_t CR3;
	uint32_t GTPR;

}USART_RegDef_t;

#define USART1					((USART_RegDef_t*)USART1_BASE)
#define USART2					((USART_RegDef_t*)USART2_BASE)
#define USART3					((USART_RegDef_t*)USART3_BASE)
#define UART4					((USART_RegDef_t*)UART4_BASE)
#define UART5					((USART_RegDef_t*)UART5_BASE)
#define USART6					((USART_RegDef_t*)USART6_BASE)


//BASIC TIMER
typedef struct
{
	uint32_t CR1;
	uint32_t CR2;
	uint32_t Reserved1;
	uint32_t DIER;
	uint32_t SR;
	uint32_t EGR;
	uint32_t Reserved[3];
	uint32_t CNT;
	uint32_t PSC;
	uint32_t ARR;

}BasicTIM_RegDef_t;

#define TIM6				((BasicTIM_RegDef_t*)TIM6_BASE)
#define TIM7				((BasicTIM_RegDef_t*)TIM7_BASE)

//GENERAL-PURPOSE TIMER
typedef struct
{
	uint32_t CR1;
	uint32_t CR2;
	uint32_t SMCR;
	uint32_t DIER;
	uint32_t SR;
	uint32_t EGR;
	uint32_t CCMR1;
	uint32_t CCMR2;
	uint32_t CCER;
	uint32_t CNT;
	uint32_t PSC;
	uint32_t ARR;
	uint32_t Reserved1;
	uint32_t CCR1;
	uint32_t CCR2;
	uint32_t CCR3;
	uint32_t CCR4;
	uint32_t Reserved2;
	uint32_t DCR;
	uint32_t DMAR;
	uint32_t TIM2_OR;
	uint32_t TIM5_OR;

}TIM2_5_RegDef_t;

#define TIM2				((TIM2_5_RegDef_t*)TIM2_BASE)
#define TIM3				((TIM2_5_RegDef_t*)TIM3_BASE)
#define TIM4				((TIM2_5_RegDef_t*)TIM4_BASE)
#define TIM5				((TIM2_5_RegDef_t*)TIM5_BASE)

typedef struct
{
	uint32_t CR1;
	uint32_t SMCR;
	uint32_t DIER;
	uint32_t SR;
	uint32_t EGR;
	uint32_t CCMR1[2];
	uint32_t Reserved1;
	uint32_t CCER;
	uint32_t CNT;
	uint32_t PSC;
	uint32_t ARR;
	uint32_t Reserved2;
	uint32_t CCR1;
	uint32_t Reserved3;
	uint32_t OR;

}TIM10_11_13_14_RegDef_t;


#define TIM10				((TIM10_11_13_14_RegDef_t*)TIM10_BASE)
#define TIM11				((TIM10_11_13_14_RegDef_t*)TIM11_BASE)
#define TIM13				((TIM10_11_13_14_RegDef_t*)TIM13_BASE)
#define TIM14				((TIM10_11_13_14_RegDef_t*)TIM14_BASE)

typedef struct
{
	uint32_t CR1;
	uint32_t SMCR;
	uint32_t DIER;
	uint32_t SR;
	uint32_t EGR;
	uint32_t CCMR1[2];
	uint32_t Reserved1;
	uint32_t CCER;
	uint32_t CNT;
	uint32_t PSC;
	uint32_t ARR;
	uint32_t Reserved2;
	uint32_t CCR1;
	uint32_t CCR2;
}TIM9_14_RegDef_t;

#define TIM9				((TIM9_14_RegDef_t*)TIM9_BASE)
#define TIM12				((TIM9_14_RegDef_t*)TIM12_BASE)

typedef struct
{
	uint32_t CR1;
	uint32_t CR2;
	uint32_t SMCR;
	uint32_t DIER;
	uint32_t SR;
	uint32_t EGR;
	uint32_t CCMR1[2];
	uint32_t CCMR2[2];
	uint32_t CCER;
	uint32_t CNT;
	uint32_t PSC;
	uint32_t ARR;
	uint32_t RCR;
	uint32_t CCR[4];
	uint32_t BDTR;
	uint32_t DCR;
	uint32_t DMAR;

}AdvTIM_RegDef_t;

#define TIM1			((AdvTIM_RegDef_t*)TIM1_BASE)
#define TIM8			((AdvTIM_RegDef_t*)TIM8_BASE)

typedef struct
{
	uint32_t CR;
	uint32_t SWTTRIGR;
	uint32_t DHR12R1;
	uint32_t DHR12L1;
	uint32_t DHR8R1;
	uint32_t DHR12R2;
	uint32_t DHR12L2;
	uint32_t DHR8R2;
	uint32_t DHR12RD;
	uint32_t DHR12LD;
	uint32_t DHR8RD;
	uint32_t DOR1;
	uint32_t DOR2;
	uint32_t SR;
}DAC_RegDef_t;

#define DAC				((DAC_RegDef_t*)DAC_BASE)

typedef struct
{
	uint32_t SR;
	uint32_t CR1;
	uint32_t CR2;
	uint32_t SMPR1;
	uint32_t SMPR2;
	uint32_t JOFR1;
	uint32_t JOFR2;
	uint32_t JOFR3;
	uint32_t JOFR4;
	uint32_t HTR;
	uint32_t LTR;
	uint32_t SQR1;
	uint32_t SQR2;
	uint32_t SQR3;
	uint32_t JSQR;
	uint32_t JDR1;
	uint32_t JDR2;
	uint32_t JDR3;
	uint32_t JDR4;
	uint32_t DR;
	uint32_t CSR;
	uint32_t CCR;
	uint32_t CDR;

}ADC_RegDef_t;

#define ADC1		((ADC_RegDef_t*)ADC1_BASE)
#define ADC2		((ADC_RegDef_t*)ADC2_BASE)
#define ADC3		((ADC_RegDef_t*)ADC3_BASE)


//IRQ interupt request for GPIO
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

//IRQ interrupt request for SPI
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51

//IRQ interrupt request for I2C
#define IRQ_I2C1_EV			31
#define IRQ_I2C1_ER			32
#define IRQ_I2C2_EV			33
#define IRQ_I2C2_ER			34
#define IRQ_I2C3_EV			79
#define IRQ_I2C3_ER			80


//enable clock for GPIOx peripherals
#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()	(RCC->AHB1ENR |= (1 << 8))


//enable macros for I2Cx clock peripherals
#define I2C1_PLCLK_EN()	(RCC->APB1ENR |= (1 << 21))


//CLOCK ENABLE MACROS FOR spiX PERIPHERALS
#define	SPI1_PCLK_EN()	(RCC->APB2ENR |= (1 << 12))
#define	SPI2_PCLK_EN()	(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()	(RCC->APB1ENR |= (1 << 15))

//Clock enable macros for I2C peripherals
#define I2C1_PCLK_EN()	(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()	(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()	(RCC->APB1ENR |= (1 << 23))

//Clock enable macros for SYSCFG peripheral
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 <<14))

//Timer clock enable macros
#define TIM1_CLK_EN()	(RCC->APB2ENR |= (1 << 0))
#define TIM2_CLK_EN()	(RCC->APB1ENR |= (1 << 0))
#define TIM3_CLK_EN()	(RCC->APB1ENR |= (1 << 1))
#define TIM4_CLK_EN()	(RCC->APB1ENR |= (1 << 2))
#define TIM5_CLK_EN()	(RCC->APB1ENR |= (1 << 3))
#define TIM6_CLK_EN()	(RCC->APB1ENR |= (1 << 4))
#define TIM7_CLK_EN()	(RCC->APB1ENR |= (1 << 5))
#define TIM8_CLK_EN()	(RCC->APB2ENR |= (1 << 1))
#define TIM9_CLK_EN()	(RCC->APB2ENR |= (1 << 16))
#define TIM10_CLK_EN()	(RCC->APB2ENR |= (1 << 17))
#define TIM11_CLK_EN()	(RCC->APB2ENR |= (1 << 18))
#define TIM12_CLK_EN()	(RCC->APB1ENR |= (1 << 12))
#define TIM13_CLK_EN()	(RCC->APB1ENR |= (1 << 13))
#define TIM14_CLK_EN()	(RCC->APB1ENR |= (1 << 14))

//ADC clock enable macros
#define ADC1_CLK_EN()	(RCC->APB2ENR |= (1 << 8))
#define ADC2_CLK_EN()	(RCC->APB2ENR |= (1 << 9))
#define ADC3_CLK_EN()	(RCC->APB2ENR |= (1 << 10))

//Clock disable macros for GPIOx peripherals
#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 8))


//SPI clock disable
#define	SPI1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 12))
#define	SPI2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 15))

//Clock disable macros for I2C peripherals
#define I2C1_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 23))

//Timer clock disable macros
#define TIM1_CLK_DI()	(RCC->APB2ENR &= ~(1 << 0))
#define TIM2_CLK_DI()	(RCC->APB1ENR &= ~(1 << 0))
#define TIM3_CLK_DI()	(RCC->APB1ENR &= ~(1 << 1))
#define TIM4_CLK_DI()	(RCC->APB1ENR &= ~(1 << 2))
#define TIM5_CLK_DI()	(RCC->APB1ENR &= ~(1 << 3))
#define TIM6_CLK_DI()	(RCC->APB1ENR &= ~(1 << 4))
#define TIM7_CLK_DI()	(RCC->APB1ENR &= ~(1 << 5))
#define TIM8_CLK_DI()	(RCC->APB2ENR &= ~(1 << 1))
#define TIM9_CLK_DI()	(RCC->APB2ENR &= ~(1 << 16))
#define TIM10_CLK_DI()	(RCC->APB2ENR &= ~(1 << 17))
#define TIM11_CLK_DI()	(RCC->APB2ENR &= ~(1 << 18))
#define TIM12_CLK_DI()	(RCC->APB1ENR &= ~(1 << 12))
#define TIM13_CLK_DI()	(RCC->APB1ENR &= ~(1 << 13))
#define TIM14_CLK_DI()	(RCC->APB1ENR &= ~(1 << 14))

//ADC clock disable macros
#define ADC1_CLK_DI()	(RCC->APB2ENR &= ~(1 << 8))
#define ADC2_CLK_DI()	(RCC->APB2ENR &= ~(1 << 9))
#define ADC3_CLK_DI()	(RCC->APB2ENR &= ~(1 << 10))


//GPIO reset maco's
#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 <<  0)); }while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 <<  1)); }while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 <<  2)); }while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 <<  3)); }while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 <<  4)); }while(0)
#define GPIOF_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 <<  5)); }while(0)
#define GPIOG_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 <<  6)); }while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 <<  7)); }while(0)
#define GPIOI_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 <<  8)); }while(0)

#define SPI1_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)

#define I2C1_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); }while(0)


//base configuration number for EXTICR bit setting
#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOF) ? 5 :\
										(x == GPIOG) ? 6 :\
										(x == GPIOH) ? 7 :\
										(x == GPIOI) ? 8 :0)


#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE

#define INPUT			0
#define OUTPUT			1
#define ALTFUNC			2
#define ANALOG			3
#define INTERRUPT		4

//bit positoin macros for SPI
#define CPHA			0
#define CPOL			1

#define FLAG_RESET		RESET
#define FLAG_SET		SET

#define BUSY			1
#define FREE			0

#endif /* INC_STM32F407XX_H_ */
