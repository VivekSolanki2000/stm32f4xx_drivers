/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: SPI.h
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Vivek Solanki
* DESCRIPTION 	: SPI configuration functions
*/
/*********************************************************************************************/
#ifndef INC_SPI_H_
#define INC_SPI_H_

/************************ Includes *************************/
#include "sysUtil.h"
#include "stm32f411xx.h"
/************************ defines *************************/
/*
 * Macro to enable SPI
 * __HANDLE__ specifies the SPI Handle.
 */
#define SPI_ENABLE(__HANDLE__)  			SET_REG_BIT((__HANDLE__)->pSPIx->CR1, SPI_CR1_SPE_POS)

/*
 * Macro to enable SPI
 * __HANDLE__ specifies the SPI Handle.
 */
#define SPI_DISABLE(__HANDLE__) 			CLR_REG_BIT((__HANDLE__)->pSPIx->CR1, SPI_CR1_SPE_POS)

/*
 * Macro Function To enable Specific Interrupt
 * __HANDLE__ specifies the SPI Handle.
 * __INTERRUPT__ specifies the interrupt source to enable.
 *         This parameter can be one of the following values:
 *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
 *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
 *            @arg SPI_IT_ERR: Error interrupt enable
 */
#define SPI_ENABLE_IT(__HANDLE__, __INTERRUPT__)   SET_REG_BIT((__HANDLE__)->pSPIx->CR2, (__INTERRUPT__))

/*
 * Macro Function To disable Specific Interrupt
 * __HANDLE__ specifies the SPI Handle.
 * __INTERRUPT__ specifies the interrupt source to enable.
 *         This parameter can be one of the following values:
 *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
 *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
 *            @arg SPI_IT_ERR: Error interrupt enable
 */
#define SPI_DISABLE_IT(__HANDLE__, __INTERRUPT__)   CLR_REG_BIT((__HANDLE__)->pSPIx->CR2, (__INTERRUPT__))

/*
 * @SPI_DeviceMode
 * SPI device mode
 * */
#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0

/*
 * @SPI_BusConfig
 * SPI Bus Configuration
 * */
#define SPI_BUSCONFIG_FD					0
#define SPI_BUSCONFIG_HD					1
#define SPI_BUSCONFIG_SIMPLEX_RXONLY		3


/*
 * @SPI_SclkSpeed
 * SPI Serial clock speed prescelar
 * */
#define SPI_BR_PRESCALAR_2					0
#define SPI_BR_PRESCALAR_4					1
#define SPI_BR_PRESCALAR_8					2
#define SPI_BR_PRESCALAR_16					3
#define SPI_BR_PRESCALAR_32					4
#define SPI_BR_PRESCALAR_64					5
#define SPI_BR_PRESCALAR_128				6
#define SPI_BR_PRESCALAR_256				7

/*
 * @SPI_DFF
 * SPI Data frame format
 * */
#define SPI_DATASIZE_8_BIT					0
#define SPI_DATASIZE_16_BIT					1

/*
 * @SPI_CPOL
 * SPI Clock Polarity
 * */
#define SPI_POLARITY_LOW					0
#define SPI_POLARITY_HIGH					1

/*
 * @SPI_CPHA
 * SPI Clock phase
 * */
#define SPI_PHASE_L_EDGE					0
#define SPI_PHASE_T_EDGE					1

/*
 * @SPI_SSM
 * SPI Slave Select management
 * */
#define SPI_NSS_HARDWARE					0
#define SPI_NSS_SOFTWARE					1

/*
 * @SPI_CR1
 * SPI CR1 register bit position macros
 * */
#define SPI_CR1_CPHA_POS					0
#define SPI_CR1_CPOL_POS					1
#define SPI_CR1_MODE_BIT_POS				2
#define SPI_CR1_BR_POS						3
#define SPI_CR1_SPE_POS						6
#define SPI_CR1_LSBFIRST_POS				7
#define SPI_CR1_SSI_POS						8
#define SPI_CR1_SSM_POS						9
#define SPI_CR1_SIMPLEC_RXONLT_BIT_POS		10
#define SPI_CR1_DFF_POS						11
#define SPI_CR1_BUSCONFIG_BIT_POS			15


/*
 * @SPI_CR2
 * SPI CR2 register interrupt bit position macros
 * */
#define CR2TXEIE							15
#define CR2RXNEIE							14
#define CR2ERRIE							13

#define SPI_IT_TXE							CR2TXEIE
#define SPI_IT_RXNE							CR2RXNEIE
#define SPI_IT_ERR							CR2ERRIE

/*
 * @SPI_SR
 * SPI SR register interrupt bit position macros
 * */
#define SR_TXE								1
#define SR_RXNE								0
/************************ typedef *************************/
/************ SPI States **************************/
typedef enum
{
	SPI_READY			= 	0x00,
	SPI_BUSY_IN_RX		=	0x01,
	SPI_BUSY_IN_TX		=	0x02
}SPI_States_e;

/**
 * Configuration structure for SPIx peripheral
 */
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


/**
 * Handle structure for SPIx (x:1,2,3,4,5) peripheral
 */
typedef struct
{
	SPI_RegDef_t	*pSPIx;		//Base address of the SPI Peripheral

	SPI_Config_t	SPICOnfig;	//SPI Configuration Structure

	/******************** Used for Interrupt Mode *******************/
	uint8_t 		*pTxBuffer;	//Store the address of the buffer which stores the data to be transmitted

	uint8_t 		*pRxBuffer;	//Store the address of the buffer which stores the received data

	uint32_t		TxLen;		//Store the Length of the data to transmitted

	uint32_t		RxLen;		//Store the Length of the data to be received

	SPI_States_e	TxState;	//Store Tx state. Possible values from SPI States

	SPI_States_e	RxState;	//Store Rx State. Possible values from SPI States

}SPI_Handle_t;

/************ SPI Flags Enum Definition ******************/
typedef enum
{
	SPI_RXNE_FLAG,
	SPI_TXE_FLAG,
	SPI_BSY_FLAG,
	SPI_OVR_FLAG,
	SPI_TXEIE_FLAG,
	SPI_RXNEIE_FLAG,
	SPI_ERRIE_FLAG
}SPI_Flag_e;

/************************ Function Declaration *************************/

/*
 * Peripheral Clock setup
 */
void SPI_PCLKControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/**
 * 	Init De-Init
 * */
void SPI_Init(SPI_Handle_t *pSPIHandle);

void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive
 */
void SPI_SendData(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t dataLen);

void SPI_ReceiveData(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t dataLen);

void SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t dataLen);

void SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t dataLen);

/*
 * IRQ Configuration and handling
 */
void SPI_IRQ_ITConfig(uint8_t IRQNumber, uint8_t EnOrDi);

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t PiIRQPriority);

void SPI_IRQHandler(SPI_Handle_t *pSPIHandle);

uint8_t SPI_GetFlagStatus(SPI_Handle_t *pSPIHandle, SPI_Flag_e flagName);

void SPI_TXInterruptHandle(SPI_Handle_t *pSPIHandle);

void SPI_RXInterruptHandle(SPI_Handle_t *pSPIHandle);

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);

#endif /* INC_SPI_H_ */
