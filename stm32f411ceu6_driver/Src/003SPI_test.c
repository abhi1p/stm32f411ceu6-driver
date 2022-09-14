/*
 * 003SPI_test.c
 *
 *  Created on: 22-Jul-2022
 *      Author: PC
 */
#include<stdio.h>
#include "stm32f411xx.h"
#include<string.h>
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

//	//NSS
//	spi.GPIO_PinConfig.GPIO_PinNo=GPIO_PIN9;
//	GPIO_Init(&spi);

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
	spi2.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	spi2.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	spi2.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	spi2.SPIConfig.SPI_SSM = SPI_SSM_EN; //enable software slave management

	SPI_Init(&spi2);
}
int main()
{
	SPI2_GPIO_INIT();	//Initialize the GPIO pin to behave as SPI2

	SPI2_INIT(); //Initialize the SPI2 peripheral

	SPI_SSI_Config(SPI2, ENABLE); //this is makes SSI bit internally high to avoid MODF fault

	SPI_PeriControl(SPI2, ENABLE);
	printf("Start transmit\n");
	char data[] = "Hello Abhishek";

	SPI_SendData(SPI2, (uint8_t*) data, strlen(data));

	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
	SPI_PeriControl(SPI2, DISABLE);
	while (1)
		;
	return 0;
}
