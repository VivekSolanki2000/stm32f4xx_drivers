/****************************************FILE DESCRIPTION**************************************/
/* FILE 		: sysUtil.h
* PROJECT 		: HAL Library for STM32F4xx
* PROGRAMMER 	: Vivek Solanki
* DESCRIPTION 	: Basic utility to get and set value
*/
/*********************************************************************************************/
#ifndef SYS_UTIL_H_
#define SYS_UTIL_H_

/***************** Includes **********************/
#include<stdio.h>

/***************** Bit Manipulation Macros **********************/
#define READ_REG(reg)							((reg))
#define WRITE_REG(reg, val)   					((reg) = 	(val))
#define CLR_REG_BIT(reg, pos)					((reg  &=  ~((1U) << (pos))))
#define SET_REG_BIT(reg, pos)					((reg  |=  ((1U) << (pos))))
#define READ_REG_BIT(reg,pos)    				((reg) &   (1U << (pos)))
#define CLR_REG_VAL(reg,clrmask,pos)			((reg) &=  ~((clrmask) << (pos)))
#define SET_REG_VAL(reg,val,setmask,pos)		do{\
													CLR_REG_VAL((reg), (setmask), (pos)); \
													((reg) |= (((val) & (setmask)) << (pos))); \
												  } while (0)
#endif
