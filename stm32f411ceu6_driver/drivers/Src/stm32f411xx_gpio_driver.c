/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: 17-Jul-2022
 *      Author: PC
 */

#include"stm32f411xx_gpio_driver.h"

/*
 * @fn					-GPIO_PeriClockControl
 *
 * @brief				-This function enable or disable peripheral clock
 *
 * @param[in]			-Base address of GPIOx peripheral
 * @param[in]			-Enable or disable macros
 *
 * @return				-None
 *
 * @note				-None
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGpIOx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pGpIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGpIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGpIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGpIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGpIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGpIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if (pGpIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGpIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGpIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGpIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGpIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGpIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}

}

/*
 * @fn					-GPIO_Init
 *
 * @brief				-This function initialized the GPIOx pin
 *
 * @param[in]			-pointer to GPIOx handle
 *
 * @return				-None
 *
 * @note				-None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

	//peripheral clock enable
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t temp1, temp2;
	temp1 = 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNo; //count to left shift
	//1. configure mode
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{

		temp2 = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) << temp1);
		pGPIOHandle->pGPIOx->MODER &= ~(3 << temp1); //clear required bit
		pGPIOHandle->pGPIOx->MODER |= temp2; //set required bit

	}
	else //interrupt
	{
		pGPIOHandle->pGPIOx->MODER &= ~(3 << temp1); //set mode as input 00-input mode

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNo;

		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FE)
		{
			//1. configure the EXTI_FTSR
			EXTI->FTSR |= (1 << temp1);
			EXTI->RTSR &= ~(1 << temp1);

		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RE)
		{
			//1. configure the EXTI_RTSR
			EXTI->RTSR |= (1 << temp1);
			EXTI->FTSR &= ~(1 << temp1);

		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_REFE)
		{
			//1. configure both EXTI_RTSR and EXTI_FTSR
			EXTI->RTSR |= (1 << temp1);
			EXTI->FTSR |= (1 << temp1);

		}

		//2.configure the GPIO port selection in SYSCFG_EXTICR
		uint32_t t1, t2;
		t1 = temp1 / 4;
		t2 = temp1 % 4;
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[t1] &= ~(0xF << (t2 * 4));
		SYSCFG->EXTICR[t1] |= (portCode << (t2 * 4));

		//3.Un-mask the line
		EXTI->IMR |= (1<<temp1);
	}

//2. configure pin speed
//	temp1 = 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNo;
	temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << temp1);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(3 << temp1); //clear bit
	pGPIOHandle->pGPIOx->OSPEEDR |= temp2;

//3. configure pin pull up/down
//	temp1 = 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNo;
	temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << temp1);
	pGPIOHandle->pGPIOx->PUPDR &= ~(3 << temp1);
	pGPIOHandle->pGPIOx->PUPDR |= temp2;

//4. configure output type
	temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNo;
	temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType << temp1);
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << temp1);
	pGPIOHandle->pGPIOx->OTYPER |= temp2;

//5. configure alternate functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) //configure only if mode is alternate functionality
	{
		uint8_t pin = pGPIOHandle->GPIO_PinConfig.GPIO_PinNo;
		temp1 = (pin % 8) * 4;
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << temp1);
		if (pin > 7)
		{
			pGPIOHandle->pGPIOx->AFRH &= ~(0xF << temp1); //clear 4 bit
			pGPIOHandle->pGPIOx->AFRH |= temp2;
		}
		else
		{
			pGPIOHandle->pGPIOx->AFRL &= ~(0xF << temp1); //clear 4 bit
			pGPIOHandle->pGPIOx->AFRL |= temp2;
		}
	}

}

/*
 * @fn					-GPIO_Deinit
 *
 * @brief				-This function de-initialized the GPIOx pin
 *
 * @param[in]			-Base address of GPIOx peripheral
 *
 * @return				-None
 *
 * @note				-None
 */
void GPIO_Deinit(GPIO_RegDef_t *pGpIOx)
{
	if (pGpIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGpIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGpIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGpIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGpIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGpIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/*
 * @fn					-GPIO_ReadFromInputPin
 *
 * @brief				-This read data from input pin
 *
 * @param[in]			-Base address of GPIOx peripheral
 * @param[in]			-Pin No. to read from
 *
 * @return				-1 or 0
 *
 * @note				-LSB is only data
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGpIOx, uint8_t pinNo)
{
	uint8_t value;
	value = (uint8_t) (pGpIOx->IDR >> pinNo) & 1;
	return value;
}

/*
 * @fn					-GPIO_ReadFromInputPort
 *
 * @brief				-This function read data from input port
 *
 * @param[in]			-Base address of GPIOx peripheral
 *
 * @return				-16 bit value from port
 *
 * @note				-None
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGpIOx)
{
	uint8_t value;
	value = (uint8_t) pGpIOx->IDR;
	return value;
}

/*
 * @fn					-GPIO_WriteToOutputPin
 *
 * @brief				-This function write data to a pin
 *
 * @param[in]			-Base address of GPIOx peripheral
 * @param[in]			-Pin No. to write data
 * @param[in]			-0 or 1
 *
 * @return				-None
 *
 * @note				-None
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGpIOx, uint8_t pinNo, uint8_t value)
{
	if (value == GPIO_PIN_SET)
	{
		pGpIOx->ODR |= (1 << pinNo);
	}
	else
	{
		pGpIOx->ODR &= ~(1 << pinNo);
	}

}

/*
 * @fn					-GPIO_WriteToOutputPort
 *
 * @brief				-This function write data to 16 bit port
 *
 * @param[in]			-Base address of GPIOx peripheral
 * @param[in]			-Data to write
 *
 * @return				-None
 *
 * @note				-None
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGpIOx, uint16_t value)
{
	pGpIOx->ODR = value;

}

/*
 * @fn					-GPIO_ToggleOutputPin
 *
 * @brief				-This function toggle the pin
 *
 * @param[in]			-Base address of GPIOx peripheral
 * @param[in]			-Pin No. to toggle
 *
 * @return				-None
 *
 * @note				-None
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGpIOx, uint8_t pinNo)
{
	pGpIOx->ODR ^= (1 << pinNo);

}

/*
 * @fn					-GPIO_IRQInterruptConfig
 *
 * @brief				-This function configure interrupt
 *
 * @param[in]			-IRQ No. of interrupt
 * @param[in]			-ENABLE or DISABLE
 *
 * @return				-None
 *
 * @note				-None
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (IRQNum <= 31)
		{
			//program ISER0
			*NVIC_ISER0 |= (1 << IRQNum);

		}
		else if ((IRQNum > 31) && (IRQNum <= 63))
		{
			//program ISER1
			*NVIC_ISER1 |= (1 << (IRQNum % 32));

		}
		else if ((IRQNum > 63) && (IRQNum < 96))
		{
			//program ISER2
			*NVIC_ISER2 |= (1 << (IRQNum % 64));

		}
	}
	else
	{
		if (IRQNum <= 31)
		{
			//program ICER0
			*NVIC_ICER0 |= (1 << IRQNum);

		}
		else if ((IRQNum > 31) && (IRQNum <= 63))
		{
			//program ICER1
			*NVIC_ICER1 |= (1 << (IRQNum % 32));

		}
		else if ((IRQNum > 63) && (IRQNum < 96))
		{
			//program ICER2
			*NVIC_ICER2 |= (1 << (IRQNum % 64));

		}
	}

}

/*
 * @fn					-GPIO_IRQInterruptConfig
 *
 * @brief				-This function configure interrupt
 *
 * @param[in]			-IRQ No. of interrupt
 * @param[in]			-Interrupt priority
 * @param[in]			-ENABLE or DISABLE
 *
 * @return				-None
 *
 * @note				-None
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority)
{
	uint8_t iprNo = IRQNum / 4;
	uint8_t byteOffstet = IRQNum % 4;
	uint8_t shiftAmount = (8 * byteOffstet) + (8 - NoOfBitImplemented);
	*(NVIC_IPR0 + iprNo) |= (IRQPriority << shiftAmount);

}

/*
 * @fn					-GPIO_IRQHAndler
 *
 * @brief				-This function handle interrupt
 *
 * @param[in]			-Interrupt Pin No.
 *
 * @return				-None
 *
 * @note				-None
 */
void GPIO_IRQHAndler(uint8_t PinNo)
{
	if (EXTI->PR & (1 << PinNo))
	{
		EXTI->PR |= (1 << PinNo);
	}

}
