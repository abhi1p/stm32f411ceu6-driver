/*
 * 001LED_toggle.c
 *
 *  Created on: 18-Jul-2022
 *      Author: PC
 */


#include"stm32f411xx_gpio_driver.h"

void delay()
{
	for(uint32_t i=0;i<8e4;i++);
}
int main()
{
	GPIO_Handle_t GPIO_LED;

	GPIO_LED.pGPIOx=GPIOC;
	GPIO_LED.GPIO_PinConfig.GPIO_PinNo=GPIO_PIN13;
	GPIO_LED.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIO_LED.GPIO_PinConfig.GPIO_PinOpType=GPIO_OUT_TYPE_OD;
	GPIO_LED.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_LED.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIO_LED);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN13);
		delay();
	}


}
