/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: 17-Jul-2022
 *      Author: PC
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include"stm32f411xx.h"

typedef struct
{
	uint8_t GPIO_PinNo;					//possible value at @GPIO_PIN_NO
	uint8_t GPIO_PinMode;				//possible value at @GPIO_PIN_MODE
	uint8_t GPIO_PinSpeed;				//possible value at @GPIO_PIN_SPPED
	uint8_t GPIO_PinPuPdControl;		//possible value at @GPIO_PIN_PUPD
	uint8_t GPIO_PinOpType;				//possible value at @GPIO_PIN_OUT_TYPE
	uint8_t GPIO_PinAltFunMode;			//possible value at @GPIO_PIN_ALTFUN
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t *pGPIOx;				//This hold base address of the GPIO port to which pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;	//This hold pin configuration settings
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NO
 * GPIO possible pin number
 */
#define GPIO_PIN0				0
#define GPIO_PIN1				1
#define GPIO_PIN2				2
#define GPIO_PIN3				3
#define GPIO_PIN4				4
#define GPIO_PIN5				5
#define GPIO_PIN6				6
#define GPIO_PIN7				7
#define GPIO_PIN8				8
#define GPIO_PIN9				9
#define GPIO_PIN10				10
#define GPIO_PIN11				11
#define GPIO_PIN12				12
#define GPIO_PIN13				13
#define GPIO_PIN14				14
#define GPIO_PIN15				15


/*
 * @GPIO_PIN_MODE
 * GPIO possible Pin Mode
 */
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2 		//alternate functionality mode
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FE			4		//falling edge interrupt
#define GPIO_MODE_IT_RE			5		//rising edge interrupt
#define GPIO_MODE_IT_REFE		6		//rising edge and falling edge interrupt


/*
 * @GPIO_PIN_OUT_TYPE
 * GPIO possible output type
 */
#define GPIO_OUT_TYPE_PP		0		//push pull output type
#define GPIO_OUT_TYPE_OD		1		//open drain output type

/*
 * @GPIO_PIN_SPPED
 * GPIO possible output speed
 */
#define GPIO_SPEED_LOW			0		//low speed
#define GPIO_SPEED_MEDIUM		1		//medium speed
#define GPIO_SPEED_FAST			2		//fast speed
#define GPIO_SPEED_HIGH			3		//high speed

/*
 * @GPIO_PIN_PUPD
 * GPIO possible pull up/down
 */
#define GPIO_PIN_NO_PUPD		0		//no pull up/down
#define GPIO_PIN_PU				1		//pull up
#define GPIO_PIN_PD				2		//pull down
#define GPIO_PIN_PUPD			3		//pull up/down

/*
 * @GPIO_PIN_ALTFUN
 * GPIO pin possible alternate functionality
 */
#define GPIO_PIN_AF0			0
#define GPIO_PIN_AF1			1
#define GPIO_PIN_AF2			2
#define GPIO_PIN_AF3			3
#define GPIO_PIN_AF4			4
#define GPIO_PIN_AF5			5
#define GPIO_PIN_AF6			6
#define GPIO_PIN_AF7			7
#define GPIO_PIN_AF8			8
#define GPIO_PIN_AF9			9
#define GPIO_PIN_AF10			10
#define GPIO_PIN_AF11			11
#define GPIO_PIN_AF12			12
#define GPIO_PIN_AF13			13
#define GPIO_PIN_AF14			14
#define GPIO_PIN_AF15			15


/*******************************************************************************************************************************
 *										APIs supported by this driver
 ********************************************************************************************************************************/



/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGpIOx, uint8_t EnOrDi);


/*
 * GPIO init and deinit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_Deinit(GPIO_RegDef_t *pGpIOx);


/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGpIOx, uint8_t pinNo);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGpIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGpIOx, uint8_t pinNo, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGpIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGpIOx, uint8_t pinNo);


/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnOrDi); //Enable or disable
void GPIO_IRQPriorityConfig(uint8_t IRQNum,uint8_t IRQPriority);
void GPIO_IRQHAndler(uint8_t PinNo);

#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
