/*
 * stm32f411xx_gpio.h
 *
 *  Created on: 01-Sep-2022
 *      Author: PC
 */

#ifndef INC_STM32F411XX_GPIO_H_
#define INC_STM32F411XX_GPIO_H_
#include "stm32f411xx.h"
#include <stdint.h>

/*
 * GPIO pin config structure
 */
typedef struct
{
	uint8_t GPIO_PinNo;					//@PinNo
	uint8_t GPIO_PinMode;				//@PinMode
	uint8_t GPIO_PinSpeed;				//@PinSpeed
	uint8_t GPIO_PinPuPdControl;		//@PuPDControl
	uint8_t GPIO_pinOpType;				//@OpType
	uint8_t GPIO_PinAltFunMode;			//@AltfunMode

}GPIO_PinConfig_t;

/**********Handle structure for GPIO********/

typedef struct
{
	GPIO_RegDef_t *pGPIOx;				//store pointer to GPIOx
	GPIO_PinConfig_t GPIO_PinConfig;	//Store GPIO pin configuration


}GPIO_Handle_t;

/**************************************************************************************
 ********************	APIs supported by this driver ************************
 *************************************************************************************/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

void GPIO_Init(GPIO_Handle_t *pGPIO_Handle);
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx);

uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNo);
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNo, uint8_t data);
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t data);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNo);

void IRQ_InterruptConfig(uint8_t IRQNo, uint8_t EnorDi);
void IRQ_Handle(uint8_t PinNo);
void IRQ_PriorityConfig(uint8_t IRQNo, uint8_t IRQPriority);

/* @PinNo possible pin no*/
#define GPIO_PIN0					0
#define GPIO_PIN1					1
#define GPIO_PIN2					2
#define GPIO_PIN3					3
#define GPIO_PIN4					4
#define GPIO_PIN5					5
#define GPIO_PIN6					6
#define GPIO_PIN7					7
#define GPIO_PIN8					8
#define GPIO_PIN9					9
#define GPIO_PIN10					10
#define GPIO_PIN11					11
#define GPIO_PIN12					12
#define GPIO_PIN13					13
#define GPIO_PIN14					14
#define GPIO_PIN15					15

/*
 * @PinMode possible
 */
#define GPIO_MODE_IN				0
#define GPIO_MODE_OUT				1
#define GPIO_MODE_ALTFN				2
#define GPIO_MODE_ANALOG			3
#define GPIO_MODE_IT_RT				4
#define GPIO_MODE_IT_FT				5
#define GPIO_MODE_IT_RTFT			6

/*
 * @PinSpeed possible
 */
#define GPIO_SPEED_LOW				0
#define GPIO_SPEED_MEDIUM			1
#define GPIO_SPEED_FAST				2
#define GPIO_SPEED_HIGH				3

/*
 * @PuPDControl
 */
#define GPIO_NO_PUPD				0
#define GPIO_PU						1
#define GPIO_PD						2

/*
 * @OpType
 */
#define GPIO_OPTYPE_PP				0			//push pull
#define GPIO_OPTYPE_OD				1			//open drain

/*
 * @AltfunMode
 */
#define GPIO_ALTFN0					0
#define GPIO_ALTFN1					1
#define GPIO_ALTFN2					2
#define GPIO_ALTFN3					3
#define GPIO_ALTFN4					4
#define GPIO_ALTFN5					5
#define GPIO_ALTFN6					6
#define GPIO_ALTFN7					7
#define GPIO_ALTFN8					8
#define GPIO_ALTFN9					9
#define GPIO_ALTFN10				10
#define GPIO_ALTFN11				11
#define GPIO_ALTFN12				12
#define GPIO_ALTFN13				13
#define GPIO_ALTFN14				14
#define GPIO_ALTFN15				15

#endif /* INC_STM32F411XX_GPIO_H_ */
