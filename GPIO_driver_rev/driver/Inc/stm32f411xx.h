/*
 * stm32f411xx.h
 *
 *  Created on: Aug 21, 2022
 *      Author: PC
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>

#define __vo				volatile

/*Processor core specific register address*/
//ISERx base address
#define NVIC_ISER0			((uint32_t *)(0xE000E100UL))
#define NVIC_ISER1			((uint32_t *)(0xE000E100UL+0x4))
#define NVIC_ISER2			((uint32_t *)(0xE000E100UL+0x8))

//ICERx base address
#define NVIC_ICER0			((uint32_t *)(0XE000E180UL))
#define NVIC_ICER1			((uint32_t *)(0XE000E180+0x4))
#define NVIC_ICER2			((uint32_t *)(0XE000E180+0x8))

//priority base address
#define NVIC_IPR0			((uint32_t *)0xE000E400UL)
#define NVIC_IPR1			((uint32_t *)(0xE000E400UL+0x4))
#define NVIC_IPR2			((uint32_t *)(0xE000E400UL+0x8))

#define NO_OF_PRIORITY_BIT_IMPLEMENTED	4

/*
 * MCU Specific register Address
 */
#define SRAM				(0x20000000UL)				//SRAM base address
#define FLASH				(0x08000000UL)				//FLASH base address
#define ROM					(0x1FFF0000UL)				//ROM base address

/*
 * BUS base address
 */
#define APB1_baseAddr		(0x40000000UL)				//APB1 bus base address
#define	APB2_baseAddr		(0x40010000UL)				//APB2 bus base address
#define AHB1_baseAddr		(0x40020000UL)				//AHB1 bus base address
#define AHB2_baseAddr		(0x50000000UL)				//AHB2 bus base address

/*
 * APB1 peripheral base address
 */
#define TIM2_baseAddr		(APB1_baseAddr)				//TImer 2 base address
#define TIM3_baseAddr		(APB1_baseAddr+0x400)		//TImer 3 base address
#define TIM4_baseAddr		(APB1_baseAddr+0x800)		//TImer 4 base address
#define TIM5_baseAddr		(APB1_baseAddr+0xC00)		//TImer 5 base address
#define RTC_BKP_baseAddr	(APB1_baseAddr+0x2800)		//RTC & BKP Registers base address
#define WWDG_baseAddr		(APB1_baseAddr+0x2C00)		//w watch dog base address
#define IWDG_baseAddr		(APB1_baseAddr+0x3000)		//I watch dog base address
#define I2S2ext_baseAddr	(APB1_baseAddr+0x3400)		//I2S2ext base address
#define SPI2_I2S2_baseAddr	(APB1_baseAddr+0x3800)		//SPI2/I2S2 base address
#define SPI3_I2S3_baseAddr	(APB1_baseAddr+0x3C00)		//SPI3 / I2S3 base address
#define I2S3ext_baseAddr	(APB1_baseAddr+0x4000)		//I2S3ext base address
#define USART2_baseAddr		(APB1_baseAddr+0x4400)		//USART2 base address
#define I2C1_baseAddr		(APB1_baseAddr+0x5400)		//I2C1 base address
#define I2C2_baseAddr		(APB1_baseAddr+0x5800)		//I2C2 base address
#define I2C3_baseAddr		(APB1_baseAddr+0x5C00)		//I2C3 base address
#define PWR_baseAddr		(APB1_baseAddr+0x7000)		//PWR base address

/*
 *APB2 peripheral base address
 */
#define TIM1_baseAddr		(APB2_baseAddr)				//timer1 base address
#define USART1_baseAddr		(APB2_baseAddr+0x1000)		//USART1 base address
#define USART6_baseAddr		(APB2_baseAddr+0x1400)		//USART6 base address
#define ADC1_baseAddr		(APB2_baseAddr+0x2000)		//ADC1 base address
#define SDIO_baseAddr		(APB2_baseAddr+0x2C00)		//SDIO base address
#define SPI1_I2S1_baseAddr	(APB2_baseAddr+0x3000)		//SPI1/I2S1 base address
#define SPI4_I2S4_baseAddr	(APB2_baseAddr+0x3400)		//SPI4/I2S4 base address
#define SYSCFG_baseAddr		(APB2_baseAddr+0x3800)		//SYSCFG base address
#define EXTI_baseAddr		(APB2_baseAddr+0x3C00)		//EXTI base address
#define TIM9_baseAddr		(APB2_baseAddr+0x4000)		//TIM9 base address
#define TIM10_baseAddr		(APB2_baseAddr+0x4400)		//TIM10 base address
#define TIM11_baseAddr		(APB2_baseAddr+0x4800)		//TIM11 base address
#define SPI5_I2S5_baseAddr	(APB2_baseAddr+0x5000)		//SPI5/I2S5 base address

/*
 * AHB1 peripheral base address
 */
#define GPIOA_baseAddr		(AHB1_baseAddr+0x0000)		//GPIOA base address
#define GPIOB_baseAddr		(AHB1_baseAddr+0x0400)		//GPIOB base address
#define GPIOC_baseAddr		(AHB1_baseAddr+0x0800)		//GPIOC base address
#define GPIOD_baseAddr		(AHB1_baseAddr+0x0C00)		//GPIOD base address
#define GPIOE_baseAddr		(AHB1_baseAddr+0x1000)		//GPIOE base address
#define GPIOH_baseAddr		(AHB1_baseAddr+0x1C00)		//GPIOH base address
#define CRC_baseAddr		(AHB1_baseAddr+0x3000)		//CRC base address
#define RCC_baseAddr		(AHB1_baseAddr+0x3800)		//RCC base address
#define FlashInt_baseAddr	(AHB1_baseAddr+0x3C00)		//Flash interface register base address
#define DMA1_baseAddr		(AHB1_baseAddr+0x6000)		//DMA1 base address
#define DMA2_baseAddr		(AHB1_baseAddr+0x6400)		//DMA2 base address

/*
 * AHB2 peripheral base address
 */
#define USB_FS_baseAddr		(0x50000000UL)				//USB OTG FS base address


/******************************** Peripheral register definition structure******************************** */

typedef struct
{
	uint32_t MODER;
	uint32_t OTYPER;
	uint32_t OSPEEDR;
	uint32_t PUPDR;
	uint32_t IDR;
	uint32_t ODR;
	uint32_t BSRR;
	uint32_t LCKR;
	uint32_t AFR[2];
//	uint32_t AFRH;

}GPIO_RegDef_t;

/******************************RCC register definition structure*********************************************/

typedef struct
{
	uint32_t CR;
	uint32_t PLLCFGR;
	uint32_t CFGR;
	uint32_t CIR;
	uint32_t AHB1RSTR;
	uint32_t AHB2RSTR;
	uint32_t Reserved1;
	uint32_t Reserved2;
	uint32_t APB1RSTR;
	uint32_t APB2RSTR;
	uint32_t Reserved3;
	uint32_t Reserved4;
	uint32_t AHB1ENR;
	uint32_t AHB2ENR;
	uint32_t Reserved5;
	uint32_t Reserved6;
	uint32_t APB1ENR;
	uint32_t APB2ENR;
	uint32_t Reserved7;
	uint32_t Reserved8;
	uint32_t AHB1LPENR;
	uint32_t AHB2LPENR;
	uint32_t Reserved9;
	uint32_t Reserved10;
	uint32_t APB1LPENR;
	uint32_t APB2LPENR;
	uint32_t Reserved11;
	uint32_t Reserved12;
	uint32_t BDCR;
	uint32_t CSR;
	uint32_t Reserved13;
	uint32_t Reserved14;
	uint32_t SSCGR;
	uint32_t PLLI2SCFGR;
	uint32_t Reserved15;
	uint32_t DCKCFGR;
}RCC_RegDef_t;

/*
 * EXTI register definition structure
 */
typedef struct
{
	uint32_t IMR;
	uint32_t EMR;
	uint32_t RTSR;
	uint32_t FTSR;
	uint32_t SWIER;
	uint32_t PR;
}EXTI_RegDef_t;

/*
 * SYSCFG register definition structure
 */
typedef struct
{
	uint32_t MEMRMP;
	uint32_t PMC;
	uint32_t EXTICR[4];
//	uint32_t EXTICR2;
//	uint32_t EXTICR3;
//	uint32_t EXTICR4;
	uint32_t reserved[2];
	uint32_t CMPCR;
}SYSCFG_RegDef_t;

/*
 * SPI register definition structure
 */
typedef struct
{
	uint32_t CR1;
	uint32_t reserved1;
	uint32_t SR;
	uint32_t DR;
	uint32_t CRCPR;
	uint32_t RXCRCR;
	uint32_t TXCRCR;
	uint32_t I2SCFGR;
	uint32_t reserved2;
	uint32_t I2SPR;
}SPI_RegDef_t;

/*
 * peripheral definition(peripheral base address type casted to GPIO_RegDef_t*
 */

#define GPIOA				((GPIO_RegDef_t*)GPIOA_baseAddr)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_baseAddr)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_baseAddr)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_baseAddr)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_baseAddr)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_baseAddr)

#define RCC					((RCC_RegDef_t*)RCC_baseAddr)
#define EXTI				((EXTI_RegDef_t*)EXTI_baseAddr)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_baseAddr)

#define SPI1				((SPI_RegDef_t*)SPI1_I2S1_baseAddr)
#define SPI2				((SPI_RegDef_t*)SPI2_I2S2_baseAddr)
#define SPI3				((SPI_RegDef_t*)SPI3_I2S3_baseAddr)
#define SPI4				((SPI_RegDef_t*)SPI4_I2S4_baseAddr)
#define SPI5				((SPI_RegDef_t*)SPI5_I2S5_baseAddr)

/*********************************CLOCK ENABLE MACROS**************************************************/

/*clock enable macros for GPIO peripheral*/
#define GPIOA_CLK_EN()		(RCC->AHB1ENR|=(1<<0))
#define GPIOB_CLK_EN()		(RCC->AHB1ENR|=(1<<1))
#define GPIOC_CLK_EN()		(RCC->AHB1ENR|=(1<<2))
#define GPIOD_CLK_EN()		(RCC->AHB1ENR|=(1<<3))
#define GPIOE_CLK_EN()		(RCC->AHB1ENR|=(1<<4))
#define GPIOH_CLK_EN()		(RCC->AHB1ENR|=(1<<7))

/*clock enable macros for I2C peripheral*/
#define I2C1_CLK_EN()		(RCC->APB1ENR|=(1<<21))
#define I2C2_CLK_EN()		(RCC->APB1ENR|=(1<<22))
#define I2C3_CLK_EN()		(RCC->APB1ENR|=(1<<23))

/*SPI peripheral clock enable macros*/
#define SPI1_CLK_EN()		(RCC->APB2ENR|=(1<<12))
#define SPI2_CLK_EN()		(RCC->APB1ENR|=(1<<14))
#define SPI3_CLK_EN()		(RCC->APB1ENR|=(1<<15))
#define SPI4_CLK_EN()		(RCC->APB2ENR|=(1<<13))
#define SPI5_CLK_EN()		(RCC->APB2ENR|=(1<<20))

/*USART peripheral clock enable macros*/
#define USART1_CLK_EN()		(RCC->APB2ENR|=(1<<4))
#define USART2_CLK_EN()		(RCC->APB1ENR|=(1<<17))
#define USART6_CLK_EN()		(RCC->APB2ENR|=(1<<5))

/*SYSCFG peripheral clock enable macros*/
#define SYSCFG_CLK_EN()		(RCC->APB2ENR|=(1<<14))


/*********************************CLOCK DISABLE MACROS**************************************************/

/*clock disable macros for GPIO peripheral*/
#define GPIOA_CLK_DI()		(RCC->AHB1ENR&=~(1<<0))
#define GPIOB_CLK_DI()		(RCC->AHB1ENR&=~(1<<1))
#define GPIOC_CLK_DI()		(RCC->AHB1ENR&=~(1<<2))
#define GPIOD_CLK_DI()		(RCC->AHB1ENR&=~(1<<3))
#define GPIOE_CLK_DI()		(RCC->AHB1ENR&=~(1<<4))
#define GPIOH_CLK_DI()		(RCC->AHB1ENR&=~(1<<7))

/*clock disable macros for I2C peripheral*/
#define I2C1_CLK_DI()		(RCC->APB1ENR&=~(1<<21))
#define I2C2_CLK_DI()		(RCC->APB1ENR&=~(1<<22))
#define I2C3_CLK_DI()		(RCC->APB1ENR&=~(1<<23))

/*SPI peripheral clock disable macros*/
#define SPI1_CLK_DI()		(RCC->APB2ENR&=~(1<<12))
#define SPI2_CLK_DI()		(RCC->APB1ENR&=~(1<<14))
#define SPI3_CLK_DI()		(RCC->APB1ENR&=~(1<<15))
#define SPI4_CLK_DI()		(RCC->APB2ENR&=~(1<<13))
#define SPI5_CLK_DI()		(RCC->APB2ENR&=~(1<<20))

/*USART peripheral clock disable macros*/
#define USART1_CLK_DI()		(RCC->APB2ENR&=~(1<<4))
#define USART2_CLK_DI()		(RCC->APB1ENR&=~(1<<17))
#define USART6_CLK_DI()		(RCC->APB2ENR&=~(1<<5))

/*SYSCFG peripheral clock disable macros*/
#define SYSCFG_CLK_DI()		(RCC->APB2ENR&=~(1<<14))



/*GPIOx peripheral reset macros*/
#define GPIOA_REG_RST()		do{RCC->AHB1RSTR |=(1<<0);RCC->AHB1RSTR &=~(1<<0);}while(0)
#define GPIOB_REG_RST()		do{RCC->AHB1RSTR |=(1<<1);RCC->AHB1RSTR &=~(1<<1);}while(0)
#define GPIOC_REG_RST()		do{RCC->AHB1RSTR |=(1<<2);RCC->AHB1RSTR &=~(1<<2);}while(0)
#define GPIOD_REG_RST()		do{RCC->AHB1RSTR |=(1<<3);RCC->AHB1RSTR &=~(1<<3);}while(0)
#define GPIOE_REG_RST()		do{RCC->AHB1RSTR |=(1<<4);RCC->AHB1RSTR &=~(1<<4);}while(0)
#define GPIOH_REG_RST()		do{RCC->AHB1RSTR |=(1<<7);RCC->AHB1RSTR &=~(1<<7);}while(0)


/*Some generic macros*/
#define ENABLE				1
#define DISABLE				0
#define SET					1
#define RESET				0
#define GPIO_PIN_SET		1
#define GPIO_PIN_RESET		0

#include "stm32f411xx_gpio.h"





#endif /* INC_STM32F411XX_H_ */
