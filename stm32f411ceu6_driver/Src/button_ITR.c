/*
 * button_ITR.c
 *
 *  Created on: 19-Jul-2022
 *      Author: PC
 */


#include<stdio.h>
#include "stm32f411xx.h"
#include<string.h>
int main()
{
	GPIO_Handle_t led,button;
	memset(&led,0,sizeof(led));
	memset(&button,0,sizeof(button));
	led.pGPIOx=GPIOC;
	led.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUT;
	led.GPIO_PinConfig.GPIO_PinNo=GPIO_PIN13;
	led.GPIO_PinConfig.GPIO_PinOpType=GPIO_OUT_TYPE_PP;
	led.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_NO_PUPD;
	led.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_PeriClockControl(GPIOC, ENABLE);

	button.pGPIOx=GPIOA;
	button.GPIO_PinConfig.GPIO_PinNo=GPIO_PIN0;
	button.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FE;
	button.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;
	button.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&led);
	GPIO_Init(&button);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

	while(1);
}

void EXTI0_IRQHandler()
{
	printf("Interrupt\n");
	GPIO_IRQHAndler(GPIO_PIN0);
	GPIO_ToggleOutputPin(GPIOC, GPIO_PIN13);
}
