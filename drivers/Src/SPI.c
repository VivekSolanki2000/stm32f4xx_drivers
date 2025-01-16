/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: SPI.c
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Vivek Solanki
* DESCRIPTION 	: SPI configuration functions
*/
/*********************************************************************************************/
#include "SPI.h"

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 * @brief             - performs SPI clock enable or disable
 * @param[in]         - pSPIx
 * @param[in]         - EnOrDi
 * @return            - None
 * @Note              -
 *********************************************************************/

void SPI_PCLKControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
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
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
		else if(pSPIx == SPI5)
		{
			SPI5_PCLK_EN();
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
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
		else if(pSPIx == SPI5)
		{
			SPI5_PCLK_DI();
		}
	}
}


/*********************************************************************
 * @fn      		  - SPI_Init
 * @brief             - This function Inits SPI peripheral
 * @param[in]         - SPI_Handle_t *pSPIHandle
 * @return            - none
 * @Note              - none
 ********************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	/******************* Enabling the Clock ***************************/
	SPI_PCLKControl(pSPIHandle->pSPIx, ENABLE);

	/*******************Configuring the Mode **************************/
	SET_REG_VAL(pSPIHandle->pSPIx->CR1,pSPIHandle->SPICOnfig.SPI_DeviceMode,0x1,SPI_CR1_MODE_BIT_POS);

	/*******************Configuring the Direction *******************/
	SET_REG_VAL(pSPIHandle->pSPIx->CR1,pSPIHandle->SPICOnfig.SPI_BusConfig,0x1,SPI_CR1_BUSCONFIG_BIT_POS);

	if(pSPIHandle->SPICOnfig.SPI_DeviceMode == SPI_BUSCONFIG_SIMPLEX_RXONLY)
	{
		CLR_REG_BIT(pSPIHandle->pSPIx->CR1,SPI_CR1_BUSCONFIG_BIT_POS);
		SET_REG_BIT(pSPIHandle->pSPIx->CR1,SPI_CR1_SIMPLEC_RXONLT_BIT_POS);
	}

	/*******************Configuring the Data Size *******************/
	SET_REG_VAL(pSPIHandle->pSPIx->CR1,pSPIHandle->SPICOnfig.SPI_DFF,0x1,SPI_CR1_DFF_POS);

	/*******************Configuring the Clock Polarity CPOL *********/
	SET_REG_VAL(pSPIHandle->pSPIx->CR1,pSPIHandle->SPICOnfig.SPI_CPOL,0x1,SPI_CR1_CPOL_POS);

	/*******************Configuring the Clock Phase CPHA *********/
	SET_REG_VAL(pSPIHandle->pSPIx->CR1,pSPIHandle->SPICOnfig.SPI_CPHA,0x1,SPI_CR1_CPHA_POS);

	/*******************Configuring the NSS **********************/
	if(pSPIHandle->SPICOnfig.SPI_SSM == SPI_NSS_SOFTWARE)
	{
		SET_REG_VAL(pSPIHandle->pSPIx->CR1,pSPIHandle->SPICOnfig.SPI_SSM,0x1,SPI_CR1_SSM_POS); //Software Slave Management
		SET_REG_VAL(pSPIHandle->pSPIx->CR1,0x1,0x1,SPI_CR1_SSM_POS); 		//Software Slave Management
		SET_REG_VAL(pSPIHandle->pSPIx->CR1,0x1,0x1,SPI_CR1_SSI_POS);		//Setting the SSI Bit high
	}
	else if(pSPIHandle->SPICOnfig.SPI_SSM == SPI_NSS_HARDWARE)
	{
		SET_REG_VAL(pSPIHandle->pSPIx->CR1,pSPIHandle->SPICOnfig.SPI_SSM,0x1,SPI_CR1_SSM_POS); //Hardware Slave Management
	}

	/******************Setting the BaudRate Prescalar *************/
	SET_REG_VAL(pSPIHandle->pSPIx->CR1,pSPIHandle->SPICOnfig.SPI_SclkSpeed,0x7,SPI_CR1_BR_POS);
}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 * @brief             - This function DeInits SPI peripheral
 * @param[in]         - SPI_RegDef_t *pGPIOx
 * @return            - none
 * @Note              - none
 ********************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if (pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
	else if (pSPIx == SPI5)
	{
		SPI5_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 * @brief             - This function Sends the data(pTxBuffer) to outer world using SPI
 * @param[in]         - SPI_Handle_t *pSPIHandle
 * @param[in]         - uint8_t *pTxBuffer
 * @param[in]         - uint32_t dataLen
 * @return            - none
 * @Note              - Blocking Code
 ********************************************************************/
void SPI_SendData(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t dataLen)
{
	//Enabling the SPI peripheral
	SPI_ENABLE(pSPIHandle);

	while(dataLen > 0)
	{
		//1. Wait untill TXE is set
		while(SPI_GetFlagStatus(pSPIHandle,SPI_TXE_FLAG) == FLAG_RESET);

		if(pSPIHandle->SPICOnfig.SPI_DFF == SPI_DATASIZE_16_BIT)
		{
			WRITE_REG(pSPIHandle->pSPIx->DR,*(uint16_t *)pTxBuffer);
			(uint16_t *)pTxBuffer++;
			dataLen -= 2;
		}
		else if(pSPIHandle->SPICOnfig.SPI_DFF == SPI_DATASIZE_8_BIT)
		{
			WRITE_REG(pSPIHandle->pSPIx->DR,*pTxBuffer);
			pTxBuffer++;
			dataLen -= 1;
		}
	}

	SPI_DISABLE(pSPIHandle);
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 * @brief             - This function reads the data in (pRxBuffer) from outer world using SPI
 * @param[in]         - SPI_Handle_t *pSPIHandle
 * @param[in]         - uint8_t *pRxBuffer
 * @param[in]         - uint32_t dataLen
 * @return            - none
 * @Note              - Blocking Code
 ********************************************************************/
void SPI_ReceiveData(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t dataLen)
{
	//Enabling the SPI peripheral
	SPI_ENABLE(pSPIHandle);

	while(dataLen > 0)
	{
		//1. Wait un-till TRXNE is set
		while(SPI_GetFlagStatus(pSPIHandle,SPI_RXNE_FLAG) == FLAG_RESET);

		if(pSPIHandle->SPICOnfig.SPI_DFF == SPI_DATASIZE_16_BIT)
		{
			*(uint16_t *)pRxBuffer = READ_REG(pSPIHandle->pSPIx->DR);
			dataLen -= 2;
			(uint16_t *)pRxBuffer++;
		}
		else if(pSPIHandle->SPICOnfig.SPI_DFF == SPI_DATASIZE_8_BIT)
		{
			*pRxBuffer = READ_REG(pSPIHandle->pSPIx->DR);
			pRxBuffer++;
			dataLen -= 1;
		}
	}

	SPI_DISABLE(pSPIHandle);
}

void SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t dataLen)
{
	SPI_States_e state = pSPIHandle->TxState;

	if(state == SPI_READY)
	{

		// 1) save Tx buffer address and data length information in global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = dataLen;

		// 2) Mark the SPI state busy in transmission so no other code can take over this peripheral
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3) Enabling the SPI Peripheral
		SPI_ENABLE(pSPIHandle);

		// 4) Enable the TXEIE control bit to get interrupt whenever the TXE Flag is set in SR
		SPI_ENABLE_IT(pSPIHandle,SPI_IT_TXE);

		//5) Data transmission will be handled by the ISR
	}

}

void SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t dataLen)
{
	SPI_States_e state = pSPIHandle->RxState;

	if(state == SPI_READY)
	{

		// 1) save Tx buffer address and data length information in global variable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = dataLen;

		// 2) Mark the SPI state busy in transmission so no other code can take over this peripheral
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3) Enabling the SPI Peripheral
		SPI_ENABLE(pSPIHandle);

		// 4) Enable the TXEIE control bit to get interrupt whenever the TXE Flag is set in SR
		SPI_ENABLE_IT(pSPIHandle,SPI_IT_RXNE);

		//5) Data receive will be handled by the ISR
	}

}
/**
 * 	IRQ configuration and handling
 * */
/*********************************************************************
 * @fn      		  - GPIO_IRQItConfig
 * @brief             - This function configures the ISER bit for given IRQ number
 * @param[in]         - uint8_t IRQNumber
 * @param[in]         - uint8_t EnOrDi
 * @return            - none
 * @Note              - none
**********************************************************************/
void SPI_IRQ_ITConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 32 && IRQNumber <= 64)
		{
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if(IRQNumber > 65 && IRQNumber <= 96)
		{
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 32 && IRQNumber <= 64)
		{
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}
		else if(IRQNumber > 65 && IRQNumber <= 96)
		{
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 * @brief             - This function configures the given priority to given IRQ number
 * @param[in]         - uint8_t IRQNumber
 * @param[in]         - uint32_t IRQPriority
 * @return            - none
 * @Note              - none
**********************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprRegNo = IRQNumber/4;
	uint8_t iprRegSection = IRQNumber % 4;

	uint8_t totalShiftAmount = (iprRegSection * 8) + (8 - NO_PR_BITS_IMPLEMNTED);

	*(NVIC_PR_BASE_ADDR + (iprRegNo)) |= (IRQPriority << totalShiftAmount);
}

/*********************************************************************
 * @fn      		  - SPI_SendDSPI_GetFlagStatusata
 * @brief             - This function is used to check flag status of SPI status register
 * @param[in]         - SPI_Handle_t *pSPIHandle
 * @param[in]         - uint8_t *flagName
 * @return            - uint8_t status value
 * @Note              - none
 ********************************************************************/
uint8_t SPI_GetFlagStatus(SPI_Handle_t *pSPIHandle, SPI_Flag_e flagName)
{
	uint8_t status = FLAG_RESET;
	switch(flagName)
	{
		case SPI_RXNE_FLAG:
		{
			status = READ_REG_BIT(pSPIHandle->pSPIx->SR,SPI_RXNE_FLAG);
			break;
		}
		case SPI_TXE_FLAG:
		{
			status = READ_REG_BIT(pSPIHandle->pSPIx->SR,SPI_TXE_FLAG);
			break;
		}
		case SPI_BSY_FLAG:
		{
			status = READ_REG_BIT(pSPIHandle->pSPIx->SR,SPI_BSY_FLAG);
			break;
		}
		case SPI_OVR_FLAG:
		{
			status = READ_REG_BIT(pSPIHandle->pSPIx->SR,SPI_OVR_FLAG);
			break;
		}
		case SPI_TXEIE_FLAG:
		{
			status = READ_REG_BIT(pSPIHandle->pSPIx->SR,SPI_TXEIE_FLAG);
			break;
		}
		case SPI_RXNEIE_FLAG:
		{
			status = READ_REG_BIT(pSPIHandle->pSPIx->SR,SPI_RXNEIE_FLAG);
			break;
		}
		case SPI_ERRIE_FLAG:
		{
			status = READ_REG_BIT(pSPIHandle->pSPIx->SR,SPI_ERRIE_FLAG);
			break;
		}
		default:
			break;
	}
	return status;
}

void SPI_TXInterruptHandle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->SPICOnfig.SPI_DFF == SPI_DATASIZE_16_BIT)
	{
		WRITE_REG(pSPIHandle->pSPIx->DR,*(uint16_t *)pSPIHandle->pTxBuffer);
		(uint16_t *)pSPIHandle->pTxBuffer++;
		pSPIHandle->TxLen-= 2;
	}
	else if(pSPIHandle->SPICOnfig.SPI_DFF == SPI_DATASIZE_8_BIT)
	{
		WRITE_REG(pSPIHandle->pSPIx->DR,*pSPIHandle->pTxBuffer);
		pSPIHandle->pTxBuffer++;
		pSPIHandle->TxLen-= 1;
	}

	if(pSPIHandle->TxLen == 0)	//When all the data is transmitted
	{
		//CLear the TXEIE Bit in the CR2 Register to diable the interrupt
		SPI_DISABLE_IT(pSPIHandle,SPI_IT_TXE);

		//Change the pointer of the Txdata to NULL
		pSPIHandle->pTxBuffer = NULL;

		//Change the Txlen to 0
		pSPIHandle->TxLen = 0;

		//Change the SPI Txstate to Ready
		pSPIHandle->TxState = SPI_READY;

		//Disable the SPI Peripheral
		SPI_DISABLE(pSPIHandle);
	}

}

void SPI_RXInterruptHandle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->SPICOnfig.SPI_DFF == SPI_DATASIZE_16_BIT)
	{
		*(uint16_t *)pSPIHandle->pRxBuffer = READ_REG(pSPIHandle->pSPIx->DR);
		pSPIHandle->RxLen -= 2;
		(uint16_t *)pSPIHandle->pRxBuffer++;
	}
	else if(pSPIHandle->SPICOnfig.SPI_DFF == SPI_DATASIZE_8_BIT)
	{
		*pSPIHandle->pRxBuffer = READ_REG(pSPIHandle->pSPIx->DR);
		pSPIHandle->pRxBuffer++;
		pSPIHandle->RxLen -= 1;
	}

	if(pSPIHandle->RxLen == 0)	//When all the data is received
	{
		//CLear the RXNEIE Bit in the CR2 Register to diable the interrupt
		SPI_DISABLE_IT(pSPIHandle,SPI_IT_RXNE);

		//Change the pointer of the Rxdata to NULL
		pSPIHandle->pRxBuffer = NULL;

		//Change the Rxlen to 0
		pSPIHandle->RxLen = 0;

		//Change the SPI Txstate to Ready
		pSPIHandle->RxState = SPI_READY;

		//Disable the SPI Peripheral
		SPI_DISABLE(pSPIHandle);
	}
}

/*********************************************************************
 * @fn      		  - SPI_IRQHandler
 * @brief             - This function is used to handle SPI interrupts
 * @param[in]         - SPI_Handle_t *pSPIHandle
 * @return            - none
 * @Note              - none
 ********************************************************************/
void SPI_IRQHandler(SPI_Handle_t *pSPIHandle)
{
	if(READ_REG_BIT(pSPIHandle->pSPIx->CR2,SPI_IT_TXE) && READ_REG_BIT(pSPIHandle->pSPIx->SR,SR_TXE))
	{
		SPI_TXInterruptHandle(pSPIHandle);
	}

	if(READ_REG_BIT(pSPIHandle->pSPIx->CR2,SPI_IT_RXNE) && READ_REG_BIT(pSPIHandle->pSPIx->SR,SR_RXNE))
	{
		SPI_RXInterruptHandle(pSPIHandle);
	}
}

