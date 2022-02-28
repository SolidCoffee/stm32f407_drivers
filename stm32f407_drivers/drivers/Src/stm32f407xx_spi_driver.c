#include "stm32f407xx_spi_driver.h"

//peripheral clock settup
void SPI_PLCK_Control(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}
			else if(pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}
	}
}

//init and deinit
void SPI_Init(SPI_Handler_t *pSPIHandle)
{

	//Configure the clock
	SPI_PLCK_Control(pSPIHandle->pSPIx, ENABLE);

	//configure  the SPI_CR1 register

	//mode configure
	pSPIHandle->pSPIx->CR1 |=  (pSPIHandle->SPI_Config.SPI_DeviceMode << 2);

	if(pSPIHandle->SPI_Config.SPI_BusConfig==FULLDUPLEX)
	{
		// bidi mode hsould be cleared
		pSPIHandle->pSPIx->CR1 &= ~(1 << 15);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig==HALFDUPLEX)
	{
		//enable  BIDI  mode
		pSPIHandle->pSPIx->CR1 |= (1 << 15);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig==SIMPLEXRX)
	{
		//BIDI mode should be cleared
		//RXONLY bitmsut be set
		pSPIHandle->pSPIx->CR1 &= ~(1 << 15);
		pSPIHandle->pSPIx->CR1 |= (1 << 10);
	}

	//Configure the clock speed
	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << 3);

	//Configure the DFF
	pSPIHandle->pSPIx->CR1 |=(pSPIHandle->SPI_Config.SPI_DFF << 11);

	//configure the CPOL
	pSPIHandle->pSPIx->CR1 |=(pSPIHandle->SPI_Config.SPI_CPOL << 1);

	//Confgiure the CPHA
	pSPIHandle->pSPIx->CR1 |=(pSPIHandle->SPI_Config.SPI_CPHA << 0);

	//Configure the SSM
	pSPIHandle->pSPIx->CR1 |=(pSPIHandle->SPI_Config.SPI_SSM << 9);
}


void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
			{
				SPI1_REG_RESET();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_REG_RESET();
			}
			else if(pSPIx == SPI3)
			{
				SPI3_REG_RESET();
			}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

//data send and receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//Wait until  the TXE is empty
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		//check the DFF bit in CR1
		if((pSPIx->CR1 & (1 << 11)))
		{
			//16 bit
			//load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//8 bit
			//loadd the data into the DR
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			//Wait until  the RXNE is empty
			while(SPI_GetFlagStatus(pSPIx,SPI_RXE_FLAG) == FLAG_RESET);

			//check the DFF bit in CR1
			if((pSPIx->CR1 & (1 << 11)))
			{
				//16 bit
				//load the data from DR to RX buffer
				*((uint16_t*)pRxBuffer) = pSPIx->DR;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}
			else
			{
				//8 bit
				//loadd the data into the DR
				*pRxBuffer = pSPIx->DR;
				Len--;
				pRxBuffer++;
			}
		}
}

//IRQ config and ISR handling
void SPI_ITConfig(uint8_t IRQNumber, uint8_t Enordi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handler_t *pHandle);


void SPI_PeripheralControlTest(SPI_Handler_t *pSPIHandle, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIHandle->pSPIx->CR1 |= (1 << 6);
	}
	else if(EnOrDi == DISABLE)
	{
		pSPIHandle->pSPIx->CR1 &= ~(1 << 6);
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		 pSPIx->CR1 |= (1 << 6);
	}
	else if(EnOrDi == DISABLE)
	{
		pSPIx->CR1 &= ~(1 << 6);
	}
}


void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
		{
			 pSPIx->CR1 |= (1 << 8);
		}
		else if(EnOrDi == DISABLE)
		{
			pSPIx->CR1 &= ~(1 << 8);
		}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
			{
				 pSPIx->CR2 |= (1 << 2);
			}
			else if(EnOrDi == DISABLE)
			{
				pSPIx->CR2 &= ~(1 << 2);
			}
}

uint8_t Busy_Check(SPI_RegDef_t *pSPIx)
{
	if(pSPIx->SR & SPI_BUSY_FLAG)
	{
		return BUSY;
	}

	return FREE;
}

void Clear_and_Push(SPI_RegDef_t *pSPIx, uint32_t Len)
{

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	//read the RX buffer to clear it
	SPI_ReceiveData(pSPIx, &dummy_read,Len);

	//sending dummy byte to flush our the response from the slave
	SPI_SendData(pSPIx,&dummy_write,1);
}
