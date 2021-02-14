/*
 * stm32f4xx_gpio_drivers.c
 *
 *  Created on: 20-Dec-2020
 *      Author: Shubham
 */

#include "stm32f4xx_gpio_driver.h"

//Periphela clock
void GPIO_PeriClockControl(GPIO_Regdef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
				{
					GPIOA_PCLK_DI();
				}else if (pGPIOx == GPIOB)
				{
					GPIOB_PCLK_DI();
				}else if (pGPIOx == GPIOC)
				{
					GPIOC_PCLK_DI();
				}else if (pGPIOx == GPIOD)
				{
					GPIOD_PCLK_DI();
				}else if (pGPIOx == GPIOE)
				{
					GPIOE_PCLK_DI();
				}else if (pGPIOx == GPIOF)
				{
					GPIOF_PCLK_DI();
				}else if (pGPIOx == GPIOG)
				{
					GPIOH_PCLK_DI();
				}else if (pGPIOx == GPIOI)
				{
					GPIOI_PCLK_DI();
				}

	}
}

//Init and De-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;   //temp.register

	//enable the peripheral clock

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1.configure the mode gpio pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));   //multiplied by 2 because in mode register every pin takes 2 bits

		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing the bits

		pGPIOHandle->pGPIOx->MODER |= temp;      //setting & stored the value of temp in the actual register

		//We should only touch those pin positions in which v r interested, so v r using bitwise OR istead of equating it with temp.
		//MODE and other below registers is physical registers it should be not affetced by other bits


	}else
	{

		//Interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//clear the corresponding FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);



		}
		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);


		//3. Enable the exti interrupt delivery using Iterrupt Mask Delivery(IMR)
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;    //reinitialize temp to zero

	//2.Configure the speed

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//3. Configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//4. Configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPtype << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//5. Configure the alt function registers.
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// configure the alternate function registers.
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] |= (0xF << (4 * temp2));  //0xF because there are 4 bit fields
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));

	}



}
void GPIO_DeInit(GPIO_Regdef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	        {
				GPIOA_REG_RESET();
			}else if (pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}else if (pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}else if (pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET();
			}else if (pGPIOx == GPIOE)
			{
				GPIOE_REG_RESET();
			}else if (pGPIOx == GPIOF)
			{
				GPIOF_REG_RESET();
			}else if (pGPIOx == GPIOG)
			{
				GPIOH_REG_RESET();
			}else if (pGPIOx == GPIOI)
			{
				GPIOI_REG_RESET();
			}

}

//Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_Regdef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;     //return 0 or 1

}
uint16_t GPIO_ReadFromInputPort(GPIO_Regdef_t *pGPIOx) //there are 16 port and it returns the value
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;   //return the entire port

}
void GPIO_WritetoOutputPin(GPIO_Regdef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}
void GPIO_WritetoOutputPort(GPIO_Regdef_t *pGPIOx, uint16_t Value)  //uint16_t because there are 16 ports
{
	pGPIOx->ODR = Value;

}
void GPIO_ToggleOutputPin(GPIO_Regdef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);     //Bitwise XOR(^=)

}

//IRQ Configuration and ISR Handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);

		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}
		else if(IRQNumber > 64 && IRQNumber < 96)
		{
			//program ISER2
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program for ICER0
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program for ICER1
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >  64 && IRQNumber < 96)
		{
			//program for ICER2
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}

}

void GPIO_PriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = 	(8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);

}
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}

}

