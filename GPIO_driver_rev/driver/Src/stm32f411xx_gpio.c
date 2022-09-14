/*
 * stm32f411xx_gpio.c
 *
 *  Created on: 01-Sep-2022
 *      Author: PC
 */


#include "stm32f411xx_gpio.h"

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi)
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_CLK_EN();
		}
		else if(pGPIOx==GPIOB)
		{
			GPIOB_CLK_EN();
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_CLK_EN();
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_CLK_EN();
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_CLK_EN();
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_CLK_EN();
		}
	}
	else
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_CLK_DI();
		}
		else if(pGPIOx==GPIOB)
		{
			GPIOB_CLK_DI();
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_CLK_DI();
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_CLK_DI();
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_CLK_DI();
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_CLK_DI();
		}
	}

}

void GPIO_Init(GPIO_Handle_t *pGPIO_Handle)
{
	pGPIO_Handle->pGPIOx->MODER &= ~(3<<(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo*2));
	//configure mode of GPIOx
	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_ANALOG)
	{

		pGPIO_Handle->pGPIOx->MODER |=(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode<<(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo*2));
	}
	else
	{
		//set mode as input
//		pGPIO_Handle->pGPIOx->MODER &=~(3<<(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo*2));
		SYSCFG_CLK_EN();

		//unmask interrupt for pin
		EXTI->IMR |=(1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo);

		//select RT/FT for interrupt
		if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |=(1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo);
			EXTI->FTSR &=~(1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo);
		}
		else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |=(1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo);
			EXTI->RTSR &=~(1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo);
		}
		else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RTFT)
		{
			EXTI->RTSR |=(1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo);
			EXTI->FTSR |=(1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo);
		}

		//select port for EXTIx interrupt line
		uint8_t t1=pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo/4;
		uint8_t t2=(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo%4)*4;

		SYSCFG->EXTICR[t1] &=~(0xF<<t2);
		if(pGPIO_Handle->pGPIOx==GPIOB)
		{
			SYSCFG->EXTICR[t1] |=(1<<t2);
		}
		else if(pGPIO_Handle->pGPIOx==GPIOC)
		{
			SYSCFG->EXTICR[t1] |=(2<<t2);
		}
		else if(pGPIO_Handle->pGPIOx==GPIOD)
		{
			SYSCFG->EXTICR[t1] |=(3<<t2);
		}
		else if(pGPIO_Handle->pGPIOx==GPIOE)
		{
			SYSCFG->EXTICR[t1] |=(4<<t2);
		}
		else if(pGPIO_Handle->pGPIOx==GPIOH)
		{
			SYSCFG->EXTICR[t1] |=(7<<t2);
		}


	}

	//configure speed of GPIOx
	pGPIO_Handle->pGPIOx->OSPEEDR &= ~(3<<(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo*2));
	pGPIO_Handle->pGPIOx->OSPEEDR |=(pGPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed<<(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo*2));

	//configure pull up/down
	pGPIO_Handle->pGPIOx->PUPDR &=~(3<<(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo*2));
	pGPIO_Handle->pGPIOx->PUPDR |=(pGPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdControl<<(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo*2));

	//configure output type
	pGPIO_Handle->pGPIOx->OTYPER &=~(1<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo);
	pGPIO_Handle->pGPIOx->OTYPER |=(pGPIO_Handle->GPIO_PinConfig.GPIO_pinOpType<<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo);

	//configure alternate functionalities
	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_ALTFN)
	{
		uint8_t t1=pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo/8;
		uint8_t t2=(pGPIO_Handle->GPIO_PinConfig.GPIO_PinNo%8)*4;
		pGPIO_Handle->pGPIOx->AFR[t1] &=~(0xF<<t2);
		pGPIO_Handle->pGPIOx->AFR[t1] |=(pGPIO_Handle->GPIO_PinConfig.GPIO_PinAltFunMode<<t2);
	}


}
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx==GPIOA)
	{
		GPIOA_REG_RST();
	}
	else if(pGPIOx==GPIOB)
	{
		GPIOB_REG_RST();
	}
	else if(pGPIOx==GPIOC)
	{
		GPIOC_REG_RST();
	}
	else if(pGPIOx==GPIOD)
	{
		GPIOD_REG_RST();
	}
	else if(pGPIOx==GPIOE)
	{
		GPIOE_REG_RST();
	}
	else if(pGPIOx==GPIOH)
	{
		GPIOH_REG_RST();
	}

}

uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNo)
{
	uint8_t data=(pGPIOx->IDR>>PinNo)&1;
	return data;
}
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx)
{
	return pGPIOx->IDR;
}

void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNo, uint8_t data)
{
	if(data==GPIO_PIN_SET)
	{
		pGPIOx->ODR |=(1<<PinNo);
	}
	else if(data==GPIO_PIN_RESET)
	{
		pGPIOx->ODR &=~(1<<PinNo);
	}
}
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t data)
{
	pGPIOx->ODR=data;

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNo)
{
	pGPIOx->ODR ^=(1<<PinNo);

}

void IRQ_InterruptConfig(uint8_t IRQNo, uint8_t EnorDi)
{
	if(EnorDi)
	{
		if(IRQNo<32)
		{
			*NVIC_ISER0 |=(1<<IRQNo);
		}
		else if(IRQNo>=32 && IRQNo<64)
		{
			*NVIC_ISER1 |=(1<<IRQNo);
		}
		else if(IRQNo>=64 && IRQNo<96)
		{
			*NVIC_ISER2 |=(1<<IRQNo);
		}
	}
	else
	{
		if(IRQNo<32)
		{
			*NVIC_ICER0 |=(1<<IRQNo);
		}
		else if(IRQNo>=32 && IRQNo<64)
		{
			*NVIC_ICER1 |=(1<<IRQNo);
		}
		else if(IRQNo>=64 && IRQNo<96)
		{
			*NVIC_ICER2 |=(1<<IRQNo);
		}
	}

}
void IRQ_PriorityConfig(uint8_t IRQNo, uint8_t IRQPriority)
{
	uint8_t t1=IRQNo/4;
	uint8_t t2=(IRQNo%4);
	uint8_t shiftAmount=(t2*8) + (8-NO_OF_PRIORITY_BIT_IMPLEMENTED);

	NVIC_IPR0[t1] |=(IRQPriority<<shiftAmount);


}
void IRQ_Handle(uint8_t PinNo)
{
	if(EXTI->PR & (1<<PinNo))
	{
		EXTI->PR |=(1<<PinNo);
	}

}
