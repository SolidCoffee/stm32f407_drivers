#include "stm32f407xx_I2C_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScaler[4] = {2,4,8,16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PCLK_EN();
			}
			else if(pI2Cx == I2C2)
			{
				I2C2_PCLK_EN();
			}
			else if(pI2Cx == I2C3)
			{
				I2C3_PCLK_EN();
			}
		}
		else
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PCLK_DI();
			}
			else if(pI2Cx == I2C2)
			{
				I2C2_PCLK_DI();
			}
			else if(pI2Cx == I2C3)
			{
				I2C3_PCLK_DI();
			}
		}
}

uint32_t RCC_GetPLLOutputClock(void)
{
	return 0;//we dont use this function
}

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,SystemClk;
	uint8_t clksrc, temp, ahbp, apb1;

	clksrc = (RCC->CFGR >> 2) & 0x3;

	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}
	else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}
	else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4)
	{
		apb1 = 1;
	}
	else
	{
		apb1 = APB1_PreScaler[temp-4];
	}

	pclk1 = (SystemClk / ahbp) / apb1;

	return pclk1;
}

void I2C_init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	//eanble I2C clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//ack control bit
	pI2CHandle->pI2Cx->CR1 |= (pI2CHandle->I2C_Config.I2C_ACKControl << 10);

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//storing slave address
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SM)
	{
		//standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		//fast mode
		tempreg |= (1 << 15); //setting the mode to fast mode since it is in SM by default
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14); //setting the duty to the duty bit in the CCR register

		//calcuate the CCR value
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == DUTY2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE config
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SM)
		{
			//standard mode
			tempreg = (RCC_GetPCLK1Value() /1000000U) +1;
		}
		else
		{
			tempreg = ((RCC_GetPCLK1Value() * 300)/ 1000000000U) +1;
		}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

//Sending the data****************************************************************************************************************
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << 8);
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}



static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; //moving the 7 bit address to the left by 1
	SlaveAddr &= ~(1 << 0); // clearing the first bit (read write bit)
	pI2Cx->DR = SlaveAddr; //writing the data to the DR which will send it to the slave

}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << 0))
	{
		if(pI2CHandle->TxRxState == RXBUSY)
		{
			if(pI2CHandle->RxSize == 1)
			{
				//1. disable ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//clear the ADDR flag(read SR1 and SR2
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}
		else
		{
			//clear the ADDR flag(read SR1 and SR2
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag(read SR1 and SR2
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << 9);
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//create the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Check the SB flag
	while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB)));

	//Send the addres of the slaev with R/W bit set to 0 (write)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//confirm the address phase is completed by checking the ADDr flag in the SR1
	while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR)));

	//confirm the ADDR flag accoridng to its software sequence
	I2C_ClearADDRFlag(pI2CHandle);

	while(Len > 0)
	{
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG))); //waiting to see if TXE flag is set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//Wait for TXE and BTF
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG)));
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF)));

	//generate the STOP conditon
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

//**********************************************************************************************************************************

//Receive data***********************************************************************************************************************

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; //moving the 7 bit address to the left by 1
	SlaveAddr |= (1 << 0); // setting the first bit (read write bit)
	pI2Cx->DR = SlaveAddr; //writing the data to the DR which will send it to the slave
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		//enable the ACK
		pI2Cx->CR1 |= (1 << 10);
	}
	else
	{
		//disable the ACK
		pI2Cx->CR1 &= ~(1 << 10);
	}
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Check the SB flag
	while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB)));

	//Send the address of the slave with R/W bit set to 1 (read)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//Wait until address phase is completed by checking the ADDR flag in the SR1 register
	while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR)));

	//read only 1 byte from slave
	if(Len ==1)
	{
		//disable acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

		//clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait for the RXNE to become 1
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG)));

		//generate STOP conditon
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	//procedure to read data from slave when Len > 1
	if(Len >1)
	{
		//clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		for(uint32_t i = Len; i > 0; i--)
		{
			//wait for the RXNE to become 1
			while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG)));

			if(i == 2)
			{
				//disable acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//generate STOP conditon
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			//read the data from the data register into the buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;
		}
	}
	//re enable acking
	if(pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}

}


//************************************************************************************************************************************

//send data interrupt enabled*********************************************************************************************************
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != TXBUSY) && (busystate != RXBUSY))
		{
			pI2CHandle->pTxBuffer = pTxBuffer;
			pI2CHandle->TxLen = Len;
			pI2CHandle->TxRxState = TXBUSY;
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			//Implement code to Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << 10);

			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << 9);


			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << 8);


		}

		return busystate;
}

//************************************************************************************************************************************

//receive data interrupt enabled******************************************************************************************************
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != TXBUSY) && (busystate != RXBUSY))
		{
			pI2CHandle->pRxBuffer = pRxBuffer;
			pI2CHandle->RxLen = Len;
			pI2CHandle->TxRxState = RXBUSY;
			pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			//Implement code to Generate START Condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << 10);

			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << 9);


			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << 8);

		}

		return busystate;
}

//************************************************************************************************************************************
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}

}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
			{
				if(IRQNumber <= 31)
				{
					//program ISER0 register
					*NVIC_ISER0 |= ( 1 << IRQNumber );

				}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
				{
					//program ISER1 register
					*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
				}
				else if(IRQNumber >= 64 && IRQNumber < 96 )
				{
					//program ISER2 register //64 to 95
					*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
				}
			}else
			{
				if(IRQNumber <= 31)
				{
					//program ICER0 register
					*NVIC_ICER0 |= ( 1 << IRQNumber );
				}else if(IRQNumber > 31 && IRQNumber < 64 )
				{
					//program ICER1 register
					*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
				}
				else if(IRQNumber >= 6 && IRQNumber < 96 )
				{
					//program ICER2 register
					*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
				}
			}
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//fin the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8- NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << 0);
	}
	else if(EnorDi == DISABLE)
	{
		pI2Cx->CR1 &= ~(1 << 0);
	}
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//interrupt handling for both maste rand slave mode of a device
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << 10);//check ITEVTEN
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << 9);//check ITBUFEN

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 0);//CHECK SB

	//1. handle for interrupt generated by SB event
	//Note: SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//SB flag is set
		//this block will not be executed in slave mode because the SB is always 0 for the slave
		//in this block lets execute the address phase
		if(pI2CHandle->TxRxState == TXBUSY)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState == RXBUSY)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 1);//CHECK ADDR
	//2.Handle for interupt geenrated by ADDR event
	//Note: when master mode : address is sent
	//when slave mode : address matched with own address
	if(temp1 && temp3)
	{
		//ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 2);//CHECK BTF
	//3. handle for interrupt generated by BTF event (Byte transfer finished)
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == TXBUSY)
		{
			//make sure that TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & (1 << 7))
			{
				//BTF, TXE = 1
					if(pI2CHandle->TxLen == 0 )
					{
						//1. generate the STOP condition
						if(pI2CHandle->Sr == I2C_DISABLE_SR)
							I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

						//2. reset all the member elements of the handle structure.
						I2C_CloseSendData(pI2CHandle);

						//3. notify the application about transmission complete
						I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);

					}
			}
		}
		else if(pI2CHandle->TxRxState == RXBUSY)
		{
			;
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 4);//CHECK STOPF
	//4. handle for interrupt generated by STOPF event
	//note : stop detecton flag is applivable only salve mode
	if(temp1 && temp3)
	{
		//STOPF flag is set
		//clear the STOPF by writeing the CR1
		pI2CHandle->pI2Cx->CR1 |= 0x000;

		//notify the application that stop is detected
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 7);//CHECK TXE
	//5. handle for interrupt genrated by TXE event
	if(temp1 && temp3 && temp2)
	{
		//TXE flag is set
		//we have to do the data transmission
		if(pI2CHandle->pI2Cx->SR2 & (1 << 0))
		{
			if(pI2CHandle->TxRxState == TXBUSY)
			{
				if(pI2CHandle->TxLen > 0)
				{
					//load the data into the DR
					pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

					//decrement the TxLen
					pI2CHandle->TxLen--;

					//increment the buffer address
					pI2CHandle->pTxBuffer++;
				}
			}
		}
		else
		{
			//check to see if the slave is in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << 2))
			{
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
			}
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << 6);//CHECK RXNE
	//6.Handle for interrupt generated by RXNE event
	if(temp1 && temp3 && temp2)
	{
		//RXNE flag is set
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << 0))
		{
			if(pI2CHandle->TxRxState == RXBUSY)
			{
				//We have to do the data reception
				if(pI2CHandle->RxSize == 1)
				{
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
					pI2CHandle->RxLen--;

				}


				if(pI2CHandle->RxSize > 1)
				{
					if(pI2CHandle->RxLen == 2)
					{
						//clear the ack bit
						I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
					}

						//read DR
						*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
						pI2CHandle->pRxBuffer++;
						pI2CHandle->RxLen--;
				}

				if(pI2CHandle->RxLen == 0)
				{
					//close the I2C data reception and notify the application

					//1. generate the stop conditon
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//2. close the I2C RX
					I2C_CloseReceiveData(pI2CHandle);

					//3. notify the application
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
				}

			}
		}
	}


}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << 10);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << 9);

	pI2CHandle->TxRxState = 0;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << 10);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << 9);


	pI2CHandle->TxRxState = 0;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << 8);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << 8);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << 8);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << 9 );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << 9);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}

__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{

}
