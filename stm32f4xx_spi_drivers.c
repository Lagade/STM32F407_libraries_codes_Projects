/*
 * stm32f4xx_spi_drivers.c
 *
 *  Created on: 04-Jan-2021
 *      Author: Shubham
 */
#include "stm32f40x.h"
#include "stm32f4xx_spi_Drivers.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


/*
 * SPI Peripheral clock API
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}else if (pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}else if (pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
		}
	else
	{
		if(EnorDi == DISABLE)
			{
			  if(pSPIx == SPI1)
				{
				   SPI1_PCLK_DI();
				}else if (pSPIx == SPI2)
				{
				   SPI2_PCLK_DI();
				}else if (pSPIx == SPI3)
				{
					SPI3_PCLK_DI();
				}

			}
	}

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;       //testing the bit position of the flag
	}
	return FLAG_RESET;       //macro defined MCU specific header file
}

/*
 * SPI Init API
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//Peripheral clock enable

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// 1st configure the SPI CR1 register

	uint32_t tempreg = 0;

	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_Devicemode << SPI_CR1_MSTR;

	//2. configure the bus
	if(pSPIHandle->SPIConfig.SPI_Busconfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_Busconfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_Busconfig == SPI_BUS_CONFIG_SIMPLEX_TXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//3. Configure the SPI serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_BR;

	//5. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;        //storing the value of tempreg in CR1 register

}

/*
 * SPI Send Data API
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint8_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. check the DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF
			//1. Load the data in the data register
			pSPIx->DR = *((uint16_t*)pTXBuffer);
			Len--;
			Len--;
			(uint16_t*)pTXBuffer++;  //Pointer is incremented so that in can point it next data item for 16-bit
		}else
		{
			//8-bit DFF
			pSPIx->DR = *pTXBuffer;
			Len--;
			pTXBuffer++;      //pointing to next 1 byte data item
		}
	}
}

/*
 * SPI Receive data API
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint8_t Len)
{
	while(Len > 0)
		{
			//1. wait until TXE is set
			while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

			//2. check the DFF bit in CR1
			if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
			{
				//16 bit DFF
				//1. Load the data from data register to RX buffer
			   *((uint16_t*)pRXBuffer) = pSPIx->DR ;
				Len--;
				Len--;
				(uint16_t*)pRXBuffer++;  //Pointer is incremented so that in can point it next data item for 16-bit
			}else
			{
				//8-bit DFF
				*pRXBuffer = pSPIx->DR ;
				Len--;
				pRXBuffer++;      //pointing to next 1 byte data item
			}
		}

}

/*
 * SPI Peripheral enable API function
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*
 * SPI SSI enabling API function
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);
		}else
		{
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}

}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
		}else
		{
			pSPIx->CR1 &= ~(1 << SPI_CR2_SSOE);
		}


}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				//program ISER0
				*NVIC_ISER0 |= (1 << IRQNumber);

			}
			else if(IRQNumber > 31 && IRQNumber < 64)
			{
				//program ISER1
				*NVIC_ISER1 |= (1 << (IRQNumber % 32));

			}
			else if(IRQNumber > 64 && IRQNumber < 96)
			{
				//program ISER2
				*NVIC_ISER2 |= (1 << (IRQNumber % 64));
			}
		}else
		{
			if(IRQNumber <= 31)
			{
				//program for ICER0
				*NVIC_ICER0 |= (1 << IRQNumber);
			}
			else if(IRQNumber > 31 && IRQNumber < 64)
			{
				//program for ICER1
				*NVIC_ICER1 |= (1 << (IRQNumber % 32));
			}
			else if(IRQNumber >  64 && IRQNumber < 96)
			{
				//program for ICER2
				*NVIC_ICER2 |= (1 << (IRQNumber % 64));
			}
		}

}

void SPI_PriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = 	(8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);

}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TXState;

	if(state!= SPI_BUSY_IN_RX)
	{
		//1. Save the TX buffer and len information in some global variables
		pSPIHandle->pTXBuffer = pTxBuffer;
		pSPIHandle->TXLen = Len;

		//2. Mark the SPI state as busy in transmission so that
		//no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TXState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXNEIE);
	}
	return state;
}

uint8_t SPI_RecevingDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RXState;

		if(state!= SPI_BUSY_IN_RX)
		{
			//1. Save the RX buffer and len information in some global variables
			pSPIHandle->pRXBuffer = pRxBuffer;
			pSPIHandle->RXLen = Len;

			//2. Mark the SPI state as busy in transmission so that
			//no other code can take over same SPI peripheral until transmission is over
			pSPIHandle->RXState = SPI_BUSY_IN_RX;

			//3. Enable the RXEIE control bit to get interrupt whenever TXE flag is set in SR
			pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
		}
		return state;


}


void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1;
	uint8_t temp2;

	//first check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXNEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//Handle RXE
		spi_rxe_interrupt_handle(pHandle);
	}

	//check for ovr flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

//helper functions

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//check the DFF bit in CR1
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF
			//1. Load the data in the data register
			pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTXBuffer);
			pSPIHandle->TXLen--;
			pSPIHandle->TXLen--;
			(uint16_t*)pSPIHandle->pTXBuffer++;  //Pointer is incremented so that in can point it next data item for 16-bit
		}else
		{
			//8-bit DFF
			pSPIHandle->pSPIx->DR = *pSPIHandle->pTXBuffer;
			pSPIHandle->TXLen--;
			pSPIHandle->pTXBuffer++;
		}
	if(! pSPIHandle->TXLen)
	{
		//Tx length is zero, so close the spi transmission and inform the application that TX is over.

		//this prevent interrupts from setting up the TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventcallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}

}

static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//check the DFF bit in CR1
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF
		    //1. Load the data in the data register
			pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pRXBuffer);
			pSPIHandle->RXLen--;
			pSPIHandle->RXLen--;
			(uint16_t*)pSPIHandle->pRXBuffer++;  //Pointer is incremented so that in can point it next data item for 16-bit
		}else
		{
			//8-bit DFF
			pSPIHandle->pSPIx->DR = *pSPIHandle->pRXBuffer;
			pSPIHandle->RXLen--;
			pSPIHandle->pTXBuffer++;
		}
		if(! pSPIHandle->RXLen)
		{
			//Rx length is zero, so close the spi transmission and inform the application that RX is over.

			//this prevent interrupts from setting up the RXE flag
			SPI_CloseReception(pSPIHandle);

			SPI_ApplicationEventcallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
		}


}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TXState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;  //as we are using this temp variable
	//2. Inform the application
	SPI_ApplicationEventcallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXNEIE);
	pSPIHandle->pTXBuffer = NULL;
	pSPIHandle->TXLen = 0;
	pSPIHandle->TXState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRXBuffer = NULL;
	pSPIHandle->RXLen = 0;
	pSPIHandle->RXState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;

	(void)temp;
}

__attribute__((weak))void SPI_ApplicationEventcallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	//this is a weak a implementation, the application may override function
}
