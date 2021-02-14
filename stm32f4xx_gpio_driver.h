/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: 20-Dec-2020
 *      Author: Shubham
 */

#ifndef INC_STM32F4XX_GPIO_DRIVER_H_
#define INC_STM32F4XX_GPIO_DRIVER_H_

#include "stm32f40x.h"

//this is a configuration structure for GPIO pin
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;           //possible values from @@GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;          //possible values from @@GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPtype;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

//This is the handle structure for GPIO pin

typedef struct
{
	GPIO_Regdef_t *pGPIOx;     //This holds the address of GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;  //This holds GPIO pin Configuration settings

}GPIO_Handle_t;

//@GPIO Pin Numbers
#define GPIO_PIN_NO_0          0
#define GPIO_PIN_NO_1          1
#define GPIO_PIN_NO_2          2
#define GPIO_PIN_NO_3          3
#define GPIO_PIN_NO_4          4
#define GPIO_PIN_NO_5          5
#define GPIO_PIN_NO_6          6
#define GPIO_PIN_NO_7          7
#define GPIO_PIN_NO_8          8
#define GPIO_PIN_NO_9          9
#define GPIO_PIN_NO_10         10
#define GPIO_PIN_NO_11         11
#define GPIO_PIN_NO_12         12
#define GPIO_PIN_NO_13         13
#define GPIO_PIN_NO_14         14
#define GPIO_PIN_NO_15         15

//@GPIO_PIN_MODES
//GPIO Pin possible modes

#define GPIO_MODE_IN     0                 //Input mode
#define GPIO_MODE_OUT    1                 //Output mode
#define GPIO_MODE_ALTFN  2                 //Alternate function
#define GPIO_MODE_ANALOG 3                 //Analog mode
#define GPIO_MODE_IT_FT  4                    //Input falling edge
#define GPIO_MODE_IT_RT  5                  //Rising edge
#define GPIO_MODE_IT_RFT 6                  //Rising edge falling edge trigger

//@
//Gpio pin possible output types
#define GPIO_OP_TYPE_PP  0                   //PushPull
#define GPIO_OP_TYPE_OD  1                //Open drain

//@GPIO_PIN_SPEED
//GPIO output pins speed possible
#define GPIO_SPEED_LOW     0
#define GPIO_SPEED_MEDIUM  1
#define GPIO_SPEED_FAST    2
#define GPIO_SPEED_HIGH    3

//GPIO pin Pull up and Pull Down configuration macros
#define GPIO_NO_PUPD           0
#define GPIO_PIN_PU            1
#define GPIO_PIN_PD            2


//API's supported by this drivers

//Periphela clock
void GPIO_PeriClockControl(GPIO_Regdef_t *pGPIOx, uint8_t EnorDi);

//Init and De-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_Regdef_t *pGPIOx);

//Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_Regdef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Regdef_t *pGPIOx);  //there are 16 port and it returns the value
void GPIO_WritetoOutputPin(GPIO_Regdef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WritetoOut_putPort(GPIO_Regdef_t *pGPIOx, uint16_t Value);   //uint16_t because there are 16 ports
void GPIO_ToggleOutputPin(GPIO_Regdef_t *pGPIOx, uint8_t PinNumber);

//IRQ Configuration and ISR Handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_PriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F4XX_GPIO_DRIVER_H_ */
