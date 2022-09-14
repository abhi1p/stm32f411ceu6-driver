/*
 * stm32f411xx.h
 *
 *  Created on: Jul 14, 2022
 *      Author: PC
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_
#include<stdint.h>
#define __vo volatile

/*********************************************************************************************
 * 						Processor specific register(NVIC)
 *********************************************************************************************/

/*
 * NVIC ISERx register
 */
#define NVIC_ISER0							((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1							((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2							((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3							((__vo uint32_t*)0xE000E10C)

/*
 * NVIC ICERx register
 */
#define NVIC_ICER0							((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1							((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2							((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3							((__vo uint32_t*)0XE000E18C)

/*
 * NVIC IPRx register
 */
#define NVIC_IPR0							((__vo uint32_t*)0xE000E400)

#define NoOfBitImplemented					4

/*********************************************************************************************
 * 						Memory specific register(NVIC)
 *********************************************************************************************/

/*
 * base address of FLASH and SRAM memories
 */

#define FLASH_BASE_ADDR						0x08000000U						//FLASH memory starting address
#define SRAM_BASE_ADDRE						0x20000000U						//SRAM starting address
#define ROM_BASE_ADDRE						0x1FFF0000U						//ROM starting address

/*
 * AHBx and APBx bus peripheral base address
 */

#define PERIPHERAL_BASE_ADDR				0x40000000						//peripheral base address
#define APB1_BASE_ADDR						0x40000000						//APB1 base address
#define APB2_BASE_ADDR						0x40007400						//APB2 base address
#define AHB1_BASE_ADDR						0x40020000						//AHB1 base address
#define AHB2_BASE_ADDR						0x50000000						//AHB1 base address

/*
 * AHB1 peripheral base address
 */

#define GPIOA_BASE_ADDR						((AHB1_BASE_ADDR)+0x0000)		//GPIOA base address
#define GPIOB_BASE_ADDR						((AHB1_BASE_ADDR)+0x0400)		//GPIOB base address
#define GPIOC_BASE_ADDR						((AHB1_BASE_ADDR)+0x0800)		//GPIOC base address
#define GPIOD_BASE_ADDR						((AHB1_BASE_ADDR)+0x0C00)		//GPIOD base address
#define GPIOE_BASE_ADDR						((AHB1_BASE_ADDR)+0x1000)		//GPIOE base address
#define GPIOH_BASE_ADDR						((AHB1_BASE_ADDR)+0x1C00)		//GPIOH base address
#define RCC_BASE_ADDR						((AHB1_BASE_ADDR)+0x3800)		//GPIOH base address

/*
 * APB1 peripheral base address
 */

#define SPI2_BASE_ADDR						((APB1_BASE_ADDR)+0x3800)		//SPI2 base address
#define SPI3_BASE_ADDR						((APB1_BASE_ADDR)+0x3C00)		//SPI3 base address
#define USART2_BASE_ADDR					((APB1_BASE_ADDR)+0x4400)		//USART2 base address
#define I2C1_BASE_ADDR						((APB1_BASE_ADDR)+0x5400)		//I2C1 base address
#define I2C2_BASE_ADDR						((APB1_BASE_ADDR)+0x5800)		//I2C2 base address
#define I2C3_BASE_ADDR						((APB1_BASE_ADDR)+0x5C00)		//I2C3 base address

/*
 * APB2 peripheral base address
 */

#define USART1_BASE_ADDR					0x40011000U						//USART1 base address
#define USART6_BASE_ADDR					((USART1_BASE_ADDR)+0x0400)		//USART6 base address
#define SPI1_BASE_ADDR						0x40013000U						//SPI1 base address
#define SPI4_BASE_ADDR						((SPI1_BASE_ADDR)+0x0400)		//SPI4 base address
#define SYSCFG_BASE_ADDR					((SPI1_BASE_ADDR)+0x0800)		//SYSCFG base address
#define SPI5_BASE_ADDR						((SPI1_BASE_ADDR)+0x2000)		//SPI5 base address
#define EXTI_BASE_ADDR						((SPI1_BASE_ADDR)+0x0C00)		//EXTI base address

/*
 * peripheral register structures definition
 */
typedef struct
{
	__vo uint32_t MODER; //mode set register 							#offset: 0x00
	__vo uint32_t OTYPER; //output type register							#offset: 0x04
	__vo uint32_t OSPEEDR; //output speed register							#offset: 0x08
	__vo uint32_t PUPDR; //pull-up/pull-down register					#offset: 0x0C
	__vo uint32_t IDR; //input data register							#offset: 0x10
	__vo uint32_t ODR; //output data register							#offset: 0x14
	__vo uint32_t BSRR;	//bit set/reset register						#offset: 0x18
	__vo uint32_t LCKR;	//configuration lock register					#offset: 0x1C
	__vo uint32_t AFRL;	//alternate function low register				#offset: 0x20
	__vo uint32_t AFRH;	//alternate function high register				#offset: 0x24

} GPIO_RegDef_t;

/*
 * RCC register structures definition
 */

typedef struct
{
	__vo uint32_t CR;//RCC clock control register					#offset: 0x00
	__vo uint32_t PLLCFGR;//RCC PLL configuration register				#offset: 0x04
	__vo uint32_t CFGR;	//RCC clock configuration register				#offset: 0x08
	__vo uint32_t CIR;//RCC clock interrupt register					#offset: 0x0C
	__vo uint32_t AHB1RSTR;	//RCC AHB1 peripheral reset register			#offset: 0x10
	__vo uint32_t AHB2RSTR;	//RCC AHB2 peripheral reset register			#offset: 0x14
	__vo uint32_t Reserved1[2];
	__vo uint32_t APB1RSTR;	//RCC APB1 peripheral reset register			#offset: 0x20
	__vo uint32_t APB2RSTR;	//RCC APB2 peripheral reset register			#offset: 0x24
	__vo uint32_t Reserved2[2];
	__vo uint32_t AHB1ENR;//RCC AHB1 peripheral clock enable register 	#offset: 0x30
	__vo uint32_t AHB2ENR;//RCC AHB2 peripheral clock enable register		#offset: 0x34
	__vo uint32_t Reserved3[2];
	__vo uint32_t APB1ENR;//RCC APB1 peripheral clock enable register		#offset: 0x40
	__vo uint32_t APB2ENR;//RCC APB2 peripheral clock enable register		#offset: 0x44
	__vo uint32_t Reserved4[2];
	__vo uint32_t AHB1LPENR;//RCC AHB1 peripheral clock enable in low power mode register			#offset: 0x50
	__vo uint32_t AHB2LPENR;//RCC AHB2 peripheral clock enable in low power mode register			#offset: 0x54
	__vo uint32_t Reserved5[2];
	__vo uint32_t APB1LPENR;//RCC APB1 peripheral clock enable in low power mode register			#offset: 0x60
	__vo uint32_t APB2LPENR;//RCC APB2 peripheral clock enabled in low power mode register			#offset: 0x64
	__vo uint32_t Reserved6[2];
	__vo uint32_t BDCR;	//RCC Backup domain control register									#offset: 0x70
	__vo uint32_t CSR;//RCC clock control & status register									#offset: 0x74
	__vo uint32_t Reserved7[2];
	__vo uint32_t SSCGR;//RCC spread spectrum clock generation register							#offset: 0x80
	__vo uint32_t PLLI2SCFGR;//RCC PLLI2S configuration register										#offset: 0x84
	__vo uint32_t Reserved8;
	__vo uint32_t DCKCFGR;//RCC Dedicated Clocks Configuration Register							#offset: 0x8C

} RCC_RegDef_t;

/*
 * EXTI register structures definition
 */
typedef struct
{
	__vo uint32_t IMR;			//Interrupt mask register
	__vo uint32_t EMR;			//Event mask register
	__vo uint32_t RTSR;			//Rising trigger selection register
	__vo uint32_t FTSR;			//Falling trigger selection register
	__vo uint32_t SWIER;		//Software interrupt event register
	__vo uint32_t PR;			//Pending register

} EXTI_RegDef_t;

/*
 * SYSCFG register structures definition
 */
typedef struct
{
	__vo uint32_t MEMRMP;		//SYSCFG memory remap register
	__vo uint32_t PMC;			//SYSCFG peripheral mode configuration register
	__vo uint32_t EXTICR[4];	//SYSCFG external interrupt configuration register
	__vo uint32_t reserved[2];
	__vo uint32_t CMPCR;		//Compensation cell control register

} SYSCFG_RegDef_t;


/*
 * SPI register structures definition
 */
typedef struct
{
	__vo uint32_t CR1;			//SPI control register 1
	__vo uint32_t CR2;			//SPI control register 2
	__vo uint32_t SR;			//SPI status register
	__vo uint32_t DR;			//SPI data register
	__vo uint32_t CRCPR;		//SPI CRC polynomial register
	__vo uint32_t RXCRCR;		//SPI RX CRC register
	__vo uint32_t TXCRCR;		//SPI TX CRC register
	__vo uint32_t I2SCFGR;		//SPI_I2S configuration register
	__vo uint32_t I2SPR;		//SPI_I2S prescaler register
}SPI_RegDef_t;

/*
 * peripheral definition macros type casted to GPIO_RegDef_t*
 */

#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASE_ADDR)

/*
 * RCC macros type casted to RCC_RegDef_t *
 */

#define RCC ((RCC_RegDef_t*)RCC_BASE_ADDR)

/*
 * EXTI macros type casted to RCC_RegDef_t*
 */
#define EXTI ((EXTI_RegDef_t*)EXTI_BASE_ADDR)

/*
 * SYSCFG macros type casted to SYSCFG_RegDef_t*
 */
#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDR)


/*
 * SPI macros type casted to SPI_RegDef_t*
 */
#define SPI1		((SPI_RegDef_t*)SPI1_BASE_ADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASE_ADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASE_ADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASE_ADDR)
#define SPI5		((SPI_RegDef_t*)SPI5_BASE_ADDR)


/*
 * CLOCK ENABLE MACRO FOR GPIOx
 */

#define GPIOA_PCLK_EN()				RCC->AHB1ENR|=(1<<0)
#define GPIOB_PCLK_EN()				RCC->AHB1ENR|=(1<<1)
#define GPIOC_PCLK_EN()				RCC->AHB1ENR|=(1<<2)
#define GPIOD_PCLK_EN()				RCC->AHB1ENR|=(1<<3)
#define GPIOE_PCLK_EN()				RCC->AHB1ENR|=(1<<4)
#define GPIOH_PCLK_EN()				RCC->AHB1ENR|=(1<<7)

/*
 * CLOCK ENABLE MACRO FOR I2Cx
 */

#define I2C1_PCLK_EN()				RCC->APB1ENR|=(1<<21)
#define I2C2_PCLK_EN()				RCC->APB1ENR|=(1<<22)
#define I2C3_PCLK_EN()				RCC->APB1ENR|=(1<<23)

/*
 * CLOCK ENABLE MACRO FOR SPIx
 */

#define SPI1_PCLK_EN()				RCC->APB2ENR|=(1<<12)
#define SPI2_PCLK_EN()				RCC->APB1ENR|=(1<<14)
#define SPI3_PCLK_EN()				RCC->APB1ENR|=(1<<15)
#define SPI4_PCLK_EN()				RCC->APB2ENR|=(1<<13)
#define SPI5_PCLK_EN()				RCC->APB2ENR|=(1<<20)

/*
 * CLOCK ENABLE MACRO FOR USARTx
 */

#define USART1_PCLK_EN()			RCC->APB2ENR|=(1<<4)
#define USART2_PCLK_EN()			RCC->APB1ENR|=(1<<17)
#define USART6_PCLK_EN()			RCC->APB2ENR|=(1<<5)

/*
 * CLOCK ENABLE MACRO FOR SYSCFG
 */

#define SYSCFG_PCLK_EN()			RCC->APB2ENR|=(1<<14)

/*
 * CLOCK DISABLE MACRO FOR GPIOx
 */

#define GPIOA_PCLK_DI()				RCC->AHB1ENR&=~(1<<0)
#define GPIOB_PCLK_DI()				RCC->AHB1ENR&=~(1<<1)
#define GPIOC_PCLK_DI()				RCC->AHB1ENR&=~(1<<2)
#define GPIOD_PCLK_DI()				RCC->AHB1ENR&=~(1<<3)
#define GPIOE_PCLK_DI()				RCC->AHB1ENR&=~(1<<4)
#define GPIOH_PCLK_DI()				RCC->AHB1ENR&=~(1<<7)

/*
 * CLOCK DISABLE MACRO FOR I2Cx
 */

#define I2C1_PCLK_DI()				RCC->APB1ENR&=~(1<<21)
#define I2C2_PCLK_DI()				RCC->APB1ENR&=~(1<<22)
#define I2C3_PCLK_DI()				RCC->APB1ENR&=~(1<<23)

/*
 * CLOCK DISABLE MACRO FOR SPIx
 */

#define SPI1_PCLK_DI()				RCC->APB2ENR&=~(1<<12)
#define SPI2_PCLK_DI()				RCC->APB1ENR&=~(1<<14)
#define SPI3_PCLK_DI()				RCC->APB1ENR&=~(1<<15)
#define SPI4_PCLK_DI()				RCC->APB2ENR&=~(1<<13)
#define SPI5_PCLK_DI()				RCC->APB2ENR&=~(1<<20)

/*
 * CLOCK DISABLE MACRO FOR USARTx
 */

#define USART1_PCLK_DI()			RCC->APB2ENR&=~(1<<4)
#define USART2_PCLK_DI()			RCC->APB1ENR&=~(1<<17)
#define USART6_PCLK_DI()			RCC->APB2ENR&=~(1<<5)

/*
 * CLOCK DISABLE MACRO FOR SYSCFG
 */

#define SYSCFG_PCLK_DI()			RCC->APB2ENR&=~(1<<14)

/*
 * GPIO reset macros
 */
#define GPIOA_REG_RESET()			do{RCC->AHB1RSTR|=(1<<0); RCC->AHB1RSTR&=~(1<<0);}while(0)
#define GPIOB_REG_RESET()			do{RCC->AHB1RSTR|=(1<<1); RCC->AHB1RSTR&=~(1<<1);}while(0)
#define GPIOC_REG_RESET()			do{RCC->AHB1RSTR|=(1<<2); RCC->AHB1RSTR&=~(1<<2);}while(0)
#define GPIOD_REG_RESET()			do{RCC->AHB1RSTR|=(1<<3); RCC->AHB1RSTR&=~(1<<3);}while(0)
#define GPIOE_REG_RESET()			do{RCC->AHB1RSTR|=(1<<4); RCC->AHB1RSTR&=~(1<<4);}while(0)
#define GPIOH_REG_RESET()			do{RCC->AHB1RSTR|=(1<<7); RCC->AHB1RSTR&=~(1<<7);}while(0)

#define GPIO_BASEADDR_TO_CODE(x)	(	(x==GPIOA)?0:\
										(x==GPIOB)?1:\
										(x==GPIOC)?2:\
										(x==GPIOD)?3:\
										(x==GPIOE)?4:\
										(x==GPIOH)?7:0)

/*
 * IRQ no for GPIO peripheral
 */
#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_EXTI15_10			40

/*
 * some generic macros
 */
#define ENABLE						1
#define DISABLE						0
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET
#define FLAG_RESET					RESET
#define FLAG_SET					SET

/***************************************************************************************************************
 * 									Bit position definition of SPI peripheral
 ***************************************************************************************************************/
/*
 * SPI control register 1 bit macros
 */
#define SPI_CR1_CPHA_BIT				0
#define SPI_CR1_CPOL_BIT				1
#define SPI_CR1_MSTR_BIT				2
#define SPI_CR1_BR_BIT					3
#define SPI_CR1_SPE_BIT					6
#define SPI_CR1_LSB_FIRST_BIT			7
#define SPI_CR1_SSI_BIT					8
#define SPI_CR1_SSM_BIT					9
#define SPI_CR1_RX_ONLY_BIT				10
#define SPI_CR1_DFF_BIT					11
#define SPI_CR1_CRC_NEXT_BIT			12
#define SPI_CR1_CRC_EN_BIT				13
#define SPI_CR1_BIDI_OE_BIT				14
#define SPI_CR1_BIDI_MODE_BIT			15

/*
 * SPI control register 2 bit macros
 */
#define SPI_CR2_RXDMAEN_BIT				0
#define SPI_CR2_TXDMAEN_BIT				1
#define SPI_CR2_SSOE_BIT				2
#define SPI_CR2_FRF_BIT					4
#define SPI_CR2_ERRIE_BIT				5
#define SPI_CR2_RXNEIE_BIT				6
#define SPI_CR2_TXEIE_BIT				7

/*
 * SPI status register bit macros
 */
#define SPI_SR_RXNE_BIT					0
#define SPI_SR_TXE_BIT					1
#define SPI_SR_CHSIDE_BIT				2
#define SPI_SR_UDR_BIT					3
#define SPI_SR_CRC_ERR_BIT				4
#define SPI_SR_MODF_BIT					5
#define SPI_SR_OVR_BIT					6
#define SPI_SR_BSY_BIT					7
#define SPI_SR_FRE_BIT					8

#include "stm32f411xx_SPI_driver.h"
#include "stm32f411xx_gpio_driver.h"
#endif /* INC_STM32F411XX_H_ */
