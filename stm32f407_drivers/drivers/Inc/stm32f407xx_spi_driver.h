#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

#include<stdint.h>

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

//device mode macros
#define MASTER			1
#define SLAVE			0

//Bus  config macors
#define FULLDUPLEX			1
#define HALFDUPLEX			2
#define SIMPLEXRX			3

//Clock Speed Macros
#define BAUDDIV2				0
#define BAUDDIV4				1
#define BAUDDIV8				2
#define BAUDDIV16				3
#define BAUDDIV32				4
#define BAUDDIV64				5
#define BAUDDIV128				6
#define BAUDDIV256				7

//DFF macros
#define EIGHTBIT				0
#define SIXTEENBIT				1

// CPOL and CPHA macros
#define HIGH					1
#define LOW						0

// SSM macros
#define SOFTWARE				1
#define HARDWARE				0

//SPI related status flag definitions
#define SPI_TXE_FLAG		(1 << 1)
#define SPI_RXE_FLAG		(1 << 0)
#define SPI_BUSY_FLAG		(1 << 7)


typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;
}SPI_Handler_t;

//peripheral clock settup
void SPI_PLCK_Control(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

//init and deinit
void SPI_Init(SPI_Handler_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

//data send and receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

//IRQ config and ISR handling
void SPI_ITConfig(uint8_t IRQNumber, uint8_t Enordi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handler_t *pHandle);

//other peripheral control API
void SPI_PeripheralControlTest(SPI_Handler_t *pSPIHandle, uint8_t EnOrDi);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t Busy_Check(SPI_RegDef_t *pSPIx);
void Clear_and_Push(SPI_RegDef_t *pSPIx, uint32_t Len);
void delay(void);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
