/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: gpio.h
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Vivek Solanki
* DESCRIPTION 	: GPIO configuration functions
*/
/*********************************************************************************************/
#ifndef GPIO_H_
#define GPIO_H_

/************************ Includes *************************/
#include "stm32f411xx.h"
#include "sysUtil.h"

/*
 * Macro function to clear the EXTI line Interrupt for particular GPIO Pin
 */
#define GPIO_EXTI_CLEAR_IT(GPIO_Pin)			(SET_REG_BIT(EXTI->PR,GPIO_Pin))

/*
 * Macro function to identify which gpio pin has raised the interrupt
 */
#define GPIO_EXTI_GET_IT(GPIO_Pin)				(READ_REG_BIT(EXTI->PR,GPIO_Pin))

/*
 * Macro function to disable Exti interrupt mask
 */
#define GPIO_EXTI_DISABLE_IT_MASK(GPIO_Pin)		(CLR_REG_VAL(EXTI->IMR,GPIO_Pin))

/*
 * Macro function to enable Exti interrupt mask
 */
#define GPIO_EXTI_ENABLE_IT_MASK(GPIO_Pin)		(SET_REG_BIT(EXTI->IMR,GPIO_Pin))


/************************ defines *************************/
/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0  							0
#define GPIO_PIN_NO_1  							1
#define GPIO_PIN_NO_2  							2
#define GPIO_PIN_NO_3  							3
#define GPIO_PIN_NO_4  							4
#define GPIO_PIN_NO_5  							5
#define GPIO_PIN_NO_6  							6
#define GPIO_PIN_NO_7  							7
#define GPIO_PIN_NO_8  							8
#define GPIO_PIN_NO_9  							9
#define GPIO_PIN_NO_10 				 			10
#define GPIO_PIN_NO_11 							11
#define GPIO_PIN_NO_12  						12
#define GPIO_PIN_NO_13 							13
#define GPIO_PIN_NO_14 							14
#define GPIO_PIN_NO_15 							15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN							0x0
#define GPIO_MODE_OUT							0x1
#define GPIO_MODE_ALTFN							0x2
#define GPIO_MODE_ANALOG						0x3
#define GPIO_MODE_IT_FT							0x4
#define GPIO_MODE_IT_RT							0x5
#define GPIO_MODE_IT_RFT						0x6

/*
 * @GPIO_PIN_OP_TYPES
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP							0x0
#define GPIO_OP_TYPE_OD							0x1

/*
 * @GPIO_PIN_SPEED
 * GPIO Pin possible output speed
 * */
#define GPIO_OP_SPEED_LOW						0x0
#define GPIO_OP_SPEED_MEDIUM					0x1
#define GPIO_OP_SPEED_HIGH						0x2
#define GPIO_OP_SPEED_FAST						0x3

/*
 * @GPIO_PIN_PD_PU
 * GPIO Pin possible pull up pull down configuration macros
 * */
#define GPIO_NO_PDPU							0x0
#define GPIO_PIN_PU								0x1
#define GPIO_PIN_PD								0x2

/************************ typedef *************************/
typedef struct
{
	uint8_t GPIO_PinNumber;					/*!< possible values from @GPIO_PIN_NUMBERS >*/
	uint8_t GPIO_PinMode;					/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;					/*!< possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;			/*!< possible values from @GPIO_PIN_PD_PU >*/
	uint8_t GPIO_PinOpType;					/*!< possible values from @GPIO_PIN_OP_TYPES >*/
	uint8_t GPIO_PinAltFuncMode;			/*!< possible values from  >*/
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t 		*pGPIOx;				/*This holds base address of the GPIO port to which the pin belongs*/
	GPIO_PinConfig_t	GPIO_PinConfig;
}GPIO_Handle_t;

/************************ Function Declaration *************************/
/**
 * 	Peripheral clock set up
 * */
void GPIO_PCLKControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

/**
 * 	Init De-Init
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/**
 * 	Read Write
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_WritetoOutputInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);

void GPIO_WritetoOutputInputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);

void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/**
 * 	IRQ configuration and handling
 * */
void GPIO_IRQItConfig(uint8_t IRQNumber, uint8_t EnOrDi);

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t PiIRQPriority);

void GPIO_IRQHandling(uint8_t PinNumber);

void GPIO_EXTI_Callback(uint8_t GPIO_Pin);

#endif /* GPIO_H_ */
