#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

#include<stdint.h>

typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

//I2C SCLSpeed
#define I2C_SM	100000
#define I2C_FM4K	400000
#define I2C_FM2K	200000

//Device address
//we do not configure with macros, given by user

// ack control
//enable disable macros

//duty cycle
#define DUTY2 0
#define DUTY169 1

//I2C related status flags
#define I2C_TXE_FLAG		(1 << 7)
#define I2C_RXNE_FLAG		(1 << 6)
#define I2C_SB				(1 << 0)
#define I2C_ADDR			(1 << 1)
#define I2C_BTF				(1 << 2)
#define I2C_ADD10			(1 << 3)
#define I2C_STOPF			(1 << 4)
#define I2C_BERR			(1 << 8)
#define I2C_ARLO			(1 << 9)
#define I2C_AF				(1 << 10)
#define I2C_OVR				(1 << 11)
#define I2C_TIMEOUT			(1 << 14)

//I2C state
#define READY				0
#define RXBUSY				1
#define TXBUSY				2

//application events
#define I2C_EV_TX_CMPLT		0
#define I2C_EV_STOP			1
#define I2C_EV_DATA_REQ		2
#define I2C_EV_RX_CMPLT    	3

#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET

#define I2C_ERROR_BERR  3
#define I2C_ERROR_ARLO  4
#define I2C_ERROR_AF    5
#define I2C_ERROR_OVR   6
#define I2C_ERROR_TIMEOUT 7

#define I2C_SR1_AF		10
#define I2C_SR1_OVR		11
#define I2C_SR1_TIMEOUT	14


typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer;   	//to store the app. Tx buffer address
	uint8_t *pRxBuffer;		//to store the app. Rx Buffewr address
	uint32_t TxLen;			//to store Tx len
	uint32_t RxLen;			//to store Rx len
	uint8_t TxRxState;		//to stoe communicaton state
	uint8_t DevAddr;		//to store slave/device address
	uint32_t RxSize;		//to store Rx size
	uint8_t Sr;   			//to store repeated start value

}I2C_Handle_t;

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

void I2C_init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);


#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
