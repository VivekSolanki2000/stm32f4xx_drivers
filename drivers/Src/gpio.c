/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: gpio.c
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Vivek Solanki
* DESCRIPTION 	: GPIO configuration functions
*/
/*********************************************************************************************/
#include "gpio.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 * @param[in]         - GPIO_RegDef_t *pGPIOx
 * @param[in]         - uint8_t EnOrDi
 * @return            - none
 * @Note              - none
**********************************************************************/
void GPIO_PCLKControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
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
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
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
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}

	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 * @brief             - This function is used to configure the GPIO
 * @param[in]         - GPIO_Handle_t *pGPIOHandle
 * @return            - none
 * @Note              - none
 **********************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < GPIO_PIN_NO_0 ||  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber > GPIO_PIN_NO_15)
	{
		return;
	}

	//==> Enable the clock
	GPIO_PCLKControl(pGPIOHandle->pGPIOx,ENABLE);

	//1. configure the mode of pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		SET_REG_VAL(pGPIOHandle->pGPIOx->MODER,pGPIOHandle->GPIO_PinConfig.GPIO_PinMode,0x3,(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. Configure FTSR register
			SET_REG_VAL(EXTI->FTSR,0x1,0x1,pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. Configure RTSR register
			SET_REG_VAL(EXTI->RTSR,0x1,0x1,pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. Configure both FTSR & RTSR register
			SET_REG_VAL(EXTI->FTSR,0x1,0x1,pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			SET_REG_VAL(EXTI->RTSR,0x1,0x1,pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure port selection in SYSCFG_EXTICR
		/*
		 * Configuring the SYSCFG EXTI Registers
		 * This will tell the EXTI to listen from which port & pin and forward their interrupt
		 * to the NVIC.
		 *
		 */
		SYSCFG_PCLK_EN(); //Enabling the SYSCFG CLOCK
		uint8_t gpioPort;

		//Finding the Appropriate Port Value
		if(pGPIOHandle->pGPIOx == GPIOA)
		{
			gpioPort = 0x00;
		}
		else if(pGPIOHandle->pGPIOx == GPIOB)
		{
			gpioPort = 0x01;
		}
		else if(pGPIOHandle->pGPIOx == GPIOC)
		{
			gpioPort = 0x02;
		}
		else if(pGPIOHandle->pGPIOx == GPIOD)
		{
			gpioPort = 0x03;
		}
		else if(pGPIOHandle->pGPIOx == GPIOE)
		{
			gpioPort = 0x04;
		}
		else if(pGPIOHandle->pGPIOx ==  GPIOH)
		{
			gpioPort = 0x07;
		}

		//Making changes in the SYSCFG EXTICR Register depending on the GPIO Pin Number
		//2. configure the GPIO port selection in SYSCFG_EXTICR
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber >=0  && pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 3)
		{
			SET_REG_VAL(SYSCFG->EXTICR[0],gpioPort,0xF,(((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 4) * 4));
		}
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber >=4  && pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7)
		{
			SET_REG_VAL(SYSCFG->EXTICR[1],gpioPort,0xF,(((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 4) * 4));
		}
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber >=8  && pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 11)
		{
			SET_REG_VAL(SYSCFG->EXTICR[2],gpioPort,0xF,(((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 4) * 4));
		}
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber >=12  && pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 15)
		{
			SET_REG_VAL(SYSCFG->EXTICR[3],gpioPort,0xF,(((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 4) * 4));
		}

		//3. configure EXTI interrupt delivery using IMR
		SET_REG_VAL(EXTI->IMR,0x1,0x1,pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	//2. configure the speed
	if(pGPIOHandle->pGPIOx->OSPEEDR >= GPIO_OP_SPEED_LOW &&  pGPIOHandle->pGPIOx->OSPEEDR <= GPIO_OP_SPEED_FAST)
		SET_REG_VAL(pGPIOHandle->pGPIOx->OSPEEDR, pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed, 0x3, (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	//3. configure pu-pd settings
	if(pGPIOHandle->pGPIOx->PUPDR >= GPIO_NO_PDPU &&  pGPIOHandle->pGPIOx->OSPEEDR <= GPIO_PIN_PD)
		SET_REG_VAL(pGPIOHandle->pGPIOx->PUPDR, pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl, 0x3, (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	//4. configure the op type
	if(pGPIOHandle->pGPIOx->OTYPER >= GPIO_OP_TYPE_PP &&  pGPIOHandle->pGPIOx->OTYPER <= GPIO_OP_TYPE_OD)
		SET_REG_VAL(pGPIOHandle->pGPIOx->OTYPER, pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType, 0x1, pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 7)
		{
			// configure lower alternate function register
			SET_REG_VAL(pGPIOHandle->pGPIOx->AFR[0], pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode, 0xF, (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber > 7 && pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 15)
		{
			// configure higher alternate function register
			SET_REG_VAL(pGPIOHandle->pGPIOx->AFR[1], pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode, 0xF, ((4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber))%32));
		}
	}

}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 * @brief             -	This function resets the GPIO configuration
 * @param[in]         - GPIO_RegDef_t *pGPIOx
 * @return            - none
 * @Note              - none
**********************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/**
 * 	Read Write
 * */
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 * @brief             -	This function reads the value of given pin
 * @param[in]         - GPIO_RegDef_t *pGPIOx
 * @param[in]         - uint8_t PinNumber
 * @return            - value
 * @Note              - none
**********************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value = (uint8_t)(( pGPIOx->IDR >> PinNumber ) & 0x00000001);
	return value;
}


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 * @brief             - This function reads the value present on given port
 * @param[in]         - GPIO_RegDef_t *pGPIOx
 * @return            - value
 * @Note              - none
**********************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value = (uint16_t)(pGPIOx->IDR );
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WritetoOutputInputPin
 * @brief             - This function writes the value on given pin number of given port
 * @param[in]         - GPIO_RegDef_t *pGPIOx
 * @param[in]         - uint8_t PinNumber
 * @param[in]         - uint8_t Port
 * @param[in]         - uint8_t value
 * @return            - none
 * @Note              - none
**********************************************************************/
void GPIO_WritetoOutputInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_WritetoOutputInputPort
 * @brief             - This function writes the value on given port
 * @param[in]         - GPIO_RegDef_t *pGPIOx
 * @param[in]         - uint16_t value
 * @return            - none
 * @Note              - none
**********************************************************************/
void GPIO_WritetoOutputInputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

/*********************************************************************
 * @fn      		  - GPIO_TogglePin
 * @brief             - This function toggles the pin value
 * @param[in]         - GPIO_RegDef_t *pGPIOx
 * @param[in]         - uint8_t PinNumber
 * @return            - none
 * @Note              - none
**********************************************************************/
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
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
void GPIO_IRQItConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprRegNo = IRQNumber/4;
	uint8_t iprRegSection = IRQNumber % 4;

	uint8_t totalShiftAmount = (iprRegSection * 8) + (8 - NO_PR_BITS_IMPLEMNTED);

	*(NVIC_PR_BASE_ADDR + (iprRegNo)) |= (IRQPriority << totalShiftAmount);
}


/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 * @brief             - This function handles the interrupt
 * @param[in]         - uint8_t PinNumber
 * @return            - none
 * @Note              - none
**********************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		//clear
		EXTI->PR |= ( 1 << PinNumber);
	}
}
