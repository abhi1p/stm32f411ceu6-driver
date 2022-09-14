/*
 * stm32f411xx_SPI_driver.c
 *
 *  Created on: 21-Jul-2022
 *      Author: PC
 */

#include "stm32f411xx_SPI_driver.h"

/*
 * @fn					-SPI_PeriClockControl
 *
 * @brief				-This function enable or disable SPI clock
 *
 * @param[in]			-Base address of SPIx peripheral
 * @param[in]			-Enable or disable macros
 *
 * @return				-None
 *
 * @note				-None
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
		else if (pSPIx == SPI5)
		{
			SPI5_PCLK_EN();
		}
	}
	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
		else if (pSPIx == SPI5)
		{
			SPI5_PCLK_DI();
		}
	}
}

/*
 * @fn					-SPI_Init
 *
 * @brief				-This function initialize the SPI peripheral
 *
 * @param[in]			-pointer to SPI_Handle_t
 *
 * @return				-None
 *
 * @note				-None
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	//spi peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//configure control register1

	uint32_t temp = 0;

	//1. configure device mode
	temp |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR_BIT);

	//2. bus configure
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//clear the bidi mode
		temp &= ~(1 << SPI_CR1_BIDI_MODE_BIT);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//set the bidi mode
		temp |= (1 << SPI_CR1_BIDI_MODE_BIT);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig
			== SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//clear the bidi mode
		temp &= ~(1 << SPI_CR1_BIDI_MODE_BIT);

		//set rxonly
		temp |= (1 << SPI_CR1_RX_ONLY_BIT);

	}

	//3. configure SCLK speed
	temp |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR_BIT);

	//4. configure spi frame format
	temp |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF_BIT);

	//5. configure CPOL
	temp |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL_BIT);

	//6. configure CPHA
	temp |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA_BIT);

	//7. configure SSM
	temp |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM_BIT);

	//initialize the CPR1
	pSPIHandle->pSPIx->CR1 = temp;

}

/*
 * @fn					-SPI_Deinit
 *
 * @brief				-This function de-initialize the SPI peripheral
 *
 * @param[in]			-pointer to SPI_RegDef_t
 *
 * @return				-None
 *
 * @note				-None
 */
void SPI_Deinit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1)
	{
		RCC->APB2RSTR |= (1 << 12);
		RCC->APB2RSTR &= ~(1 << 12);
	}
	else if (pSPIx == SPI2)
	{
		RCC->APB1RSTR |= (1 << 14);
		RCC->APB1RSTR &= ~(1 << 14);
	}
	else if (pSPIx == SPI3)
	{
		RCC->APB1RSTR |= (1 << 15);
		RCC->APB1RSTR &= ~(1 << 15);
	}
	else if (pSPIx == SPI4)
	{
		RCC->APB2RSTR |= (1 << 13);
		RCC->APB2RSTR &= ~(1 << 13);
	}
	else if (pSPIx == SPI5)
	{
		RCC->APB2RSTR |= (1 << 20);
		RCC->APB2RSTR &= ~(1 << 20);
	}

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t flagName)
{
	if (pSPIx->SR & flagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * @fn					-SPI_SendData
 *
 * @brief				-This function send data using SPI protocol
 *
 * @param[in]			-pointer to SPI_RegDef_t
 * @param[in]			-pointer to TX buffer
 * @param[in]			-size of data;
 *
 * @return				-None
 *
 * @note				-This is blocking call
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while (len > 0)
	{
		//1. wait for TXE to set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET)
			;

		//2. check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF_BIT))
		{
			//16 bit frame format
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			len -= 2;
			(uint16_t*) pTxBuffer++;
		}
		else
		{
			//8 bit frame format
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}

	}

}
/*
 * @fn					-SPI_ReceiveData
 *
 * @brief				-This function receive data using SPI protocol
 *
 * @param[in]			-pointer to SPI_RegDef_t
 * @param[in]			-pointer to RX buffer
 * @param[in]			-size of data;
 *
 * @return				-None
 *
 * @note				-This is blocking call
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while (len > 0)
	{
		//1. wait for TXE to set
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET)
			;

		//2. check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF_BIT))
		{
			//16 bit frame format
			*((uint16_t*) pRxBuffer) = pSPIx->DR; //read 16 bit data from RX data buffer
			len -= 2;
			(uint16_t*) pRxBuffer++;
		}
		else
		{
			//8 bit frame format
			*pRxBuffer = pSPIx->DR; //read 8 bit data from RX data buffer
			len--;
			pRxBuffer++;
		}

	}
}
/*
 * @fn					-SPI_PeriControl
 *
 * @brief				-This function enable or disable the SPI peripheral
 * @param[in]			-pointer to pSPIx
 * @param[in]			-ENABLE or DISABLE
 *
 * @return				-None
 *
 * @note				-None
 */
void SPI_PeriControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	pSPIx->CR1 &= ~(1 << SPI_CR1_SPE_BIT); //clear SPE bit
	pSPIx->CR1 |= (EnOrDi << SPI_CR1_SPE_BIT); //set data to SPE bit
}

/*
 * @fn					-SPI_SSI_Config
 *
 * @brief				-This function set or reset the SSI bit
 * @param[in]			-pointer to pSPIx
 * @param[in]			-ENABLE or DISABLE
 *
 * @return				-None
 *
 * @note				-None
 */
void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	pSPIx->CR1 &= ~(1 << SPI_CR1_SSI_BIT); //clear SSI bit
	pSPIx->CR1 |= (EnOrDi << SPI_CR1_SSI_BIT); //set data to SSI bit
}

/*
 * @fn					-SPI_SSOE_Config
 *
 * @brief				-This function set or reset the SSOE bit
 * @param[in]			-pointer to pSPIx
 * @param[in]			-ENABLE or DISABLE
 *
 * @return				-None
 *
 * @note				-None
 */
void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE_BIT); //clear SSI bit
	pSPIx->CR2 |= (EnOrDi << SPI_CR2_SSOE_BIT); //set data to SSI bit
}

/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnOrDi); //Enable or disable
void SPI_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority);
void SPI_IRQHAndler(SPI_Handle_t *pHandle);
;
