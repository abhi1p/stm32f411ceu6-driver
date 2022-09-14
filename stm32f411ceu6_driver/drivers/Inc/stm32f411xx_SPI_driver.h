/*
 * stm32f411xx_SPI_driver.h
 *
 *  Created on: 21-Jul-2022
 *      Author: PC
 */

#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_
#include "stm32f411xx.h"

typedef struct
{
	uint8_t SPI_DeviceMode;			//possible value at @SPI_DEVICE_MODE
	uint8_t SPI_BusConfig;			//possible value at @SPI_BUS_CONFIG
	uint8_t SPI_SclkSpeed;			//possible value at @SPI_SCLK_SPEED
	uint8_t SPI_DFF;				//possible value at @SPI_FRAME_FORMAT
	uint8_t SPI_CPOL;				//possible value at @CPOL
	uint8_t SPI_CPHA;				//possible value at @CPHA
	uint8_t SPI_SSM;
} SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
} SPI_Handle_t;

/*
 * @SPI_DEVICE_MODE
 */
#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0

/*
 * @SPI_BUS_CONFIG
 */
#define SPI_BUS_CONFIG_FD					0			//full duplex
#define SPI_BUS_CONFIG_HD					1			//half duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		2			//simplex rx only

/*
 * @SPI_SCLK_SPEED
 */
#define SPI_SCLK_SPEED_DIV2					0			//SCLK divided by 2
#define SPI_SCLK_SPEED_DIV4					1			//SCLK divided by 4
#define SPI_SCLK_SPEED_DIV8					2			//SCLK divided by 8
#define SPI_SCLK_SPEED_DIV16				3			//SCLK divided by 16
#define SPI_SCLK_SPEED_DIV32				4			//SCLK divided by 32
#define SPI_SCLK_SPEED_DIV64				5			//SCLK divided by 64
#define SPI_SCLK_SPEED_DIV128				6			//SCLK divided by 128
#define SPI_SCLK_SPEED_DIV256				7			//SCLK divided by 256

/*
 * @SPI_FRAME_FORMAT
 */
#define SPI_DFF_8BITS						0			//8 bit data format
#define SPI_DFF_16BITS						1			//16 bit data format

/*
 * @CPOL
 */
#define SPI_CPOL_HIGH						1			//clock polarity high
#define SPI_CPOL_LOW						0			//clock polarity low

/*
 * @CPHA
 */
#define SPI_CPHA_HIGH						1			//clock phase high
#define SPI_CPHA_LOW						0			//clock phase low

/*
 * @SSM
 */
#define SPI_SSM_EN							1			//slave management software enable
#define SPI_SSM_DI							0			//slave management software disable

/*
 * SPI status related flag definition
 */
#define SPI_TXE_FLAG						(1<<SPI_SR_TXE_BIT)
#define SPI_RXNE_FLAG						(1<<SPI_SR_RXNE_BIT)
#define SPI_BUSY_FLAG						(1<<SPI_SR_BSY_BIT)

/*******************************************************************************************************************************
 *										APIs supported by this driver
 ********************************************************************************************************************************/

/*
 * Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * GPIO init and deinit
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_Deinit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnOrDi); //Enable or disable
void SPI_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority);
void SPI_IRQHAndler(SPI_Handle_t *pHandle);
;

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t flagName);

void SPI_PeriControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi); //enable or disable spi peripheral
void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
