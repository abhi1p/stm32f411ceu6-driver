/*
 * 002LED_Button.c
 *
 *  Created on: 18-Jul-2022
 *      Author: PC
 */


#include"stm32f411xx_gpio_driver.h"

int main()
{
	GPIO_Handle_t led,button;

	led.pGPIOx=GPIOC;
	led.GPIO_PinConfig.GPIO_PinNo=GPIO_PIN13;
	led.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	led.GPIO_PinConfig.GPIO_PinOpType=GPIO_OUT_TYPE_OD;
	led.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	led.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_NO_PUPD;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&led);

	button.pGPIOx=GPIOA;
	button.GPIO_PinConfig.GPIO_PinNo=GPIO_PIN0;
	button.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	button.GPIO_PinConfig.GPIO_PinOpType=GPIO_OUT_TYPE_PP;
	button.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	button.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&button);


	uint8_t preVal,currVal;
	while(1)
	{
		currVal=GPIO_ReadFromInputPin(GPIOA, GPIO_PIN0);
		if((currVal==1) &&(preVal==0))
		{
			GPIO_ToggleOutputPin(GPIOC, GPIO_PIN13);
		}
		preVal=currVal;

	}


}
