/*
 * 004SPI_SendToArduino.c
 *
 *  Created on: 23-Jul-2022
 *      Author: PC
 */

/*
 * 003SPI_test.c
 *
 *  Created on: 22-Jul-2022
 *      Author: PC
 */
#include "stm32f411xx.h"
#include<string.h>
#include<stdio.h>
/*PB9	-	NSS
 *PB10	-	SCLK
 *PB14	-	MISO
 *PB15	-	MOSI
 *Alternate Functionality	-	5
 *
 */

void SPI2_GPIO_INIT()
{
	GPIO_Handle_t spi;
	spi.pGPIOx = GPIOB;
	spi.GPIO_PinConfig.GPIO_PinOpType = GPIO_OUT_TYPE_PP;
	spi.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	spi.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
	spi.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	spi.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_PIN_AF5;

	//NSS
	spi.GPIO_PinConfig.GPIO_PinNo = GPIO_PIN9;
	GPIO_Init(&spi);

//SCLK
	spi.GPIO_PinConfig.GPIO_PinNo = GPIO_PIN10;
	GPIO_Init(&spi);

//	//MISO
//	spi.GPIO_PinConfig.GPIO_PinNo=GPIO_PIN14;
//	GPIO_Init(&spi);

//MOSI
	spi.GPIO_PinConfig.GPIO_PinNo = GPIO_PIN15;
	GPIO_Init(&spi);

}

void SPI2_INIT()
{
	SPI_Handle_t spi2;
	spi2.pSPIx = SPI2;
	spi2.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	spi2.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	spi2.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	spi2.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	spi2.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	spi2.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	spi2.SPIConfig.SPI_SSM = SPI_SSM_DI; //enable software slave management

	SPI_Init(&spi2);
}
void delay()
{
	for(uint32_t i=0;i<5e5/2;i++);
}
void buttonInit()
{
	GPIO_Handle_t button;
	button.pGPIOx = GPIOA;
	button.GPIO_PinConfig.GPIO_PinNo = GPIO_PIN0;
	button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
//	button.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FE;
	button.GPIO_PinConfig.GPIO_PinOpType = GPIO_OUT_TYPE_PP;
	button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&button);
}
int main()
{
	SPI2_GPIO_INIT();	//Initialize the GPIO pin to behave as SPI2

	SPI2_INIT(); //Initialize the SPI2 peripheral

	buttonInit();
//	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

	SPI_SSOE_Config(SPI2, ENABLE);
	while (1)
	{
		while (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN0));
		delay();
		SPI_PeriControl(SPI2, ENABLE);

//		char data[] = "Hello Abhishek";
		char data[] = "Shipper created a label, UPS has not received the package yet.";

		uint8_t len=strlen(data);
		SPI_SendData(SPI2, &len, 1);
		SPI_SendData(SPI2, (uint8_t*) data, len); //send data

		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
		SPI_PeriControl(SPI2, DISABLE);
	}

	return 0;
}

#if 0
void  EXTI0_IRQHandler()
{
	printf("Button Clicked\n");
	GPIO_IRQHAndler(GPIO_PIN0);
}
#endif
