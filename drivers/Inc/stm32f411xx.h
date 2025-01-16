/*
 * stm32f411xx.h
 *
 *  Created on: Jan 6, 2025
 *      Author: Vivek
 */

#ifndef STM32F411XX_H_
#define STM32F411XX_H_

#include <stdint.h>

#define __vo volatile
#define __weak __attribute__ ((weak))
/********************************** START:Processor Specific Details **********************************/

#define NO_PR_BITS_IMPLEMNTED			  4

/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */


#define NVIC_ISER0						   ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1						   ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2						   ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3						   ((__vo uint32_t*)0xE000E10C)

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0						   ((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1						   ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2						   ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3						   ((__vo uint32_t*)0xE000E18C)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 					((__vo uint32_t*)0xE000E400)

/**
 * Base addresses of SRAM and Flash memories
 * */
#define FLASH_BASEADDR						0x08000000U
#define SRAM1_BASEADDR						0x20000000U
#define ROM_BASEADDR						0x1FFF0000U
#define SRAM								SRAM1_BASEADDR

/**
 * Base addresses of AHBx and APBx busses
 * */
#define PERIPH_BASEADDR						0x4000000U
#define APB1_PERIPH_BASEADDR				PERIPH_BASEADDR
#define APB2_PERIPH_BASEADDR				0x40010000U
#define AHB1_PERIPH_BASEADDR				0x40020000U
#define AHB2_PERIPH_BASEADDR				0x50000000U
#define RCC_BASEADDR						0x40023800U


/**
 * Base addresses of AHB1 peripherals
 * */
#define GPIOA_BASEADDR			 			(AHB1_PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				 		(AHB1_PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				 		(AHB1_PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			 			(AHB1_PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			 			(AHB1_PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR			 			(AHB1_PERIPH_BASEADDR + 0x1C00)

/**
 * Base addresses of APB1 peripherals
 * */
#define I2C1_BASEADDR						(APB1_PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1_PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1_PERIPH_BASEADDR + 0x5C00)
#define SPI2_BASE_ADDR						(APB1_PERIPH_BASEADDR + 0x3800)
#define SPI3_BASE_ADDR						(APB1_PERIPH_BASEADDR + 0x3C00)
#define USART2_BASE_ADDR					(APB1_PERIPH_BASEADDR + 0x4400)

/**
 * Base addresses of APB2 peripherals
 * */
#define EXTI_BASEADDR						(APB2_PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASE_ADDR						(AHB2_PERIPH_BASEADDR + 0x3000)
#define SPI4_BASE_ADDR						(AHB2_PERIPH_BASEADDR + 0x3400)
#define SPI5_BASE_ADDR						(AHB2_PERIPH_BASEADDR + 0x5000)
#define USART1_BASE_ADDR					(AHB2_PERIPH_BASEADDR + 0x1000)
#define USART6_BASE_ADDR					(AHB2_PERIPH_BASEADDR + 0x1400)
#define SYSCFG_BASE_ADDR					(AHB2_PERIPH_BASEADDR + 0x3800)

/**
 * GPIO Peripheral register definition structures
 * */
typedef struct{
	__vo uint32_t	MODER;				/* GPIO port mode register */
	__vo uint32_t	OTYPER;				/* GPIO port output type register */
	__vo uint32_t	OSPEEDR;			/* GPIO port output speed register */
	__vo uint32_t	PUPDR;				/* GPIO port pull-up/pull-down register  */
	__vo uint32_t	IDR;				/* GPIO port input data register */
	__vo uint32_t	ODR;				/* GPIO port output data register */
	__vo uint32_t	BSRR;				/* GPIO port bit set/reset register */
	__vo uint32_t	LCKR;				/* GPIO port configuration lock register */
	__vo uint32_t	AFR[2];				/* GPIO alternate function register AF[0] =  AFRL, AF[1] AFRH*/
}GPIO_RegDef_t;


/**
 * Peripheral definitions type casted to RegDef.
 * */
#define GPIOA								((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPIOB								((GPIO_RegDef_t *) GPIOB_BASEADDR)
#define GPIOC								((GPIO_RegDef_t *) GPIOC_BASEADDR)
#define GPIOD								((GPIO_RegDef_t *) GPIOD_BASEADDR)
#define GPIOE								((GPIO_RegDef_t *) GPIOE_BASEADDR)
#define GPIOH								((GPIO_RegDef_t *) GPIOH_BASEADDR)

/*
 * SPI peripheral register definition structure
 */
typedef struct
{
	__vo uint32_t	CR1;				/*  SPI control register 1 */
	__vo uint32_t	CR2;				/*  SPI control register 2 */
	__vo uint32_t	SR;					/*  SPI status register */
	__vo uint32_t	DR;					/*  SPI data register */
	__vo uint32_t	CRCPR;				/*  SPI CRC polynomial register */
	__vo uint32_t 	RXCRCR;				/*  SPI RX CRC register */
	__vo uint32_t 	TXCRCR;				/*  SPI TX CRC register */
	__vo uint32_t	I2SCFGR;			/*  SPI_I2S configuration register */
	__vo uint32_t 	I2SPR;				/*  SPI_I2S prescaler register */
}SPI_RegDef_t;

/**
 * Peripheral definitions type casted to RegDef.
 * */
#define SPI1								((SPI_RegDef_t *) SPI1_BASE_ADDR)
#define SPI2								((SPI_RegDef_t *) SPI2_BASE_ADDR)
#define SPI3								((SPI_RegDef_t *) SPI3_BASE_ADDR)
#define SPI4								((SPI_RegDef_t *) SPI4_BASE_ADDR)
#define SPI5								((SPI_RegDef_t *) SPI5_BASE_ADDR)

/**
 * RCC register definition structures
 * */
typedef struct{
	__vo uint32_t	CR;
	__vo uint32_t	PLLCFGR;
	__vo uint32_t	CFGR;
	__vo uint32_t	CIR;
	__vo uint32_t	AHB1RSTR;
	__vo uint32_t	AHB2RSTR;
	uint32_t		RESERVED0;
	uint32_t		RESERVED1;
	__vo uint32_t	APB1RSTR;
	__vo uint32_t	APB2RSTR;
	uint32_t		RESERVED3;
	uint32_t		RESERVED4;
	__vo uint32_t	AHB1ENR;
	__vo uint32_t	AHB2ENR;
	uint32_t		RESERVED5;
	uint32_t		RESERVED6;
	__vo uint32_t	APB1ENR;
	__vo uint32_t	APB2ENR;
	uint32_t		RESERVED7;
	uint32_t		RESERVED8;
	__vo uint32_t	AHB1LPENR;
	__vo uint32_t	AHB2LPENR;
	uint32_t		RESERVED9;
	uint32_t		RESERVED10;
	__vo uint32_t	BDCR;
	__vo uint32_t	CSR;
	uint32_t		RESERVED11;
	uint32_t		RESERVED12;
	__vo uint32_t	PLLI2SCFGR;
	uint32_t		RESERVED13;
	__vo uint32_t	DCKCFGR;
}RCC_t;


/**
 * EXTI register definition structure
 * */
typedef struct{
	__vo uint32_t	IMR;
	__vo uint32_t	EMR;
	__vo uint32_t	RTSR;
	__vo uint32_t	FTSR;
	__vo uint32_t	SWIER;
	__vo uint32_t	PR;
}EXTI_t;

/**
 * SYSCNFG register definition structure
 * */
typedef struct{
	__vo uint32_t	MEMRMP;
	__vo uint32_t	PMC;
	__vo uint32_t	EXTICR[4];
	uint32_t		RESERVED1[2];
	__vo uint32_t	CMPCR;
	uint32_t		RESERVED2[2];
	__vo uint32_t	CFGR;
}SYSCFG_t;

#define RCC 								((RCC_t*) RCC_BASEADDR)
#define EXTI								((EXTI_t*) EXTI_BASEADDR)
#define SYSCFG								((SYSCFG_t*) SYSCFG_BASE_ADDR)

/**
 * Clock enable macros for GPIOx peripherals
 * */
#define GPIOA_PCLK_EN()						(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()						(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()						(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()						(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()						(RCC->AHB1ENR |= (1<<4))
#define GPIOH_PCLK_EN()						(RCC->AHB1ENR |= (1<<7))

/**
 * Clock enable macros for I2Cx peripherals
 * */
#define I2C1_PCLK_EN()						(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()						(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()						(RCC->APB1ENR |= (1<<23))

/**
 * Clock enable macros for SPIx peripherals
 * */
#define SPI1_PCLK_EN()						(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()						(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()						(RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()						(RCC->APB2ENR |= (1<<13))
#define SPI5_PCLK_EN()						(RCC->APB2ENR |= (1<<20))

/**
 * Clock enable macros for USARTx peripherals
 * */
#define USART1_PCLK_EN()					(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()					(RCC->APB1ENR |= (1<<17))
#define USART6_PCLK_EN()					(RCC->APB2ENR |= (1<<5))

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() 					(RCC->APB2ENR |= (1 << 14))

/**
 * Clock disable macros for GPIOx peripherals
 * */
#define GPIOA_PCLK_DI()						(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()						(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()						(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()						(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()						(RCC->AHB1ENR &= ~(1<<4))
#define GPIOH_PCLK_DI()						(RCC->AHB1ENR &= ~(1<<7))

/**
 * Clock disable macros for I2Cx peripherals
 * */
#define I2C1_PCLK_DI()						(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()						(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()						(RCC->APB1ENR &= ~(1<<23))

/**
 * Clock disable macros for SPIx peripherals
 * */
#define SPI1_PCLK_DI()						(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()						(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()						(RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()						(RCC->APB2ENR &= ~(1<<13))
#define SPI5_PCLK_DI()						(RCC->APB2ENR &= ~(1<<20))

/**
 * Clock disable macros for USARTx peripherals
 * */
#define USART1_PCLK_DI()					(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()					(RCC->APB1ENR &= ~(1<<17))
#define USART6_PCLK_DI()					(RCC->APB2ENR &= ~(1<<5))

/*
 * Clock disable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI() 					(RCC->APB2ENR &= ~(1 << 14))

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()       	        do{ (RCC->AHB1RSTR |= (1 << 0));  (RCC->AHB1RSTR &= ~(1 << 0));  }while(0)
#define GPIOB_REG_RESET()           	    do{ (RCC->AHB1RSTR |= (1 << 1));  (RCC->AHB1RSTR &= ~(1 << 1));  }while(0)
#define GPIOC_REG_RESET()            		do{ (RCC->AHB1RSTR |= (1 << 2));  (RCC->AHB1RSTR &= ~(1 << 2));  }while(0)
#define GPIOD_REG_RESET()               	do{ (RCC->AHB1RSTR |= (1 << 3));  (RCC->AHB1RSTR &= ~(1 << 3));  }while(0)
#define GPIOE_REG_RESET()               	do{ (RCC->AHB1RSTR |= (1 << 4));  (RCC->AHB1RSTR &= ~(1 << 4));  }while(0)
#define GPIOH_REG_RESET()               	do{ (RCC->AHB1RSTR |= (1 << 7));  (RCC->AHB1RSTR &= ~(1 << 7));  }while(0)

#define SPI1_REG_RESET()       	    	    do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()           		    do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->AHB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()           			do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->AHB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()           	    	do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0)
#define SPI5_REG_RESET()           	    	do{ (RCC->APB2RSTR |= (1 << 20)); (RCC->APB2RSTR &= ~(1 << 20)); }while(0)

#define IRQ_NO_EXTI0 						6
#define IRQ_NO_EXTI1 						7
#define IRQ_NO_EXTI2 						8
#define IRQ_NO_EXTI3 						9
#define IRQ_NO_EXTI4 						10
#define IRQ_NO_EXTI9_5 						23
#define IRQ_NO_EXTI15_10 					40


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    					0
#define NVIC_IRQ_PRI1    					1
#define NVIC_IRQ_PRI2    					2
#define NVIC_IRQ_PRI3    					3
#define NVIC_IRQ_PRI4   					4
#define NVIC_IRQ_PRI5    					5
#define NVIC_IRQ_PRI6    					6
#define NVIC_IRQ_PRI7    					7
#define NVIC_IRQ_PRI8    					8
#define NVIC_IRQ_PRI9    					9
#define NVIC_IRQ_PRI10   					10
#define NVIC_IRQ_PRI11   					11
#define NVIC_IRQ_PRI12   					12
#define NVIC_IRQ_PRI13   					13
#define NVIC_IRQ_PRI14   					14
#define NVIC_IRQ_PRI15   					15


/*
 * Generic Macros
 */
#define ENABLE								1
#define DISABALE							0
#define SET									ENABLE
#define RESET								DISABALE
#define GPIO_PIN_SET						SET
#define GPIO_PIN_RESET						RESET
#define FLAG_RESET							RESET
#define FLAG_SET							SET

#include "gpio.h"

#endif /* STM32F411XX_H_ */
