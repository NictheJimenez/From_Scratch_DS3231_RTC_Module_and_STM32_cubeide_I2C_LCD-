/*
 * stm32l475xx_gpio_driver.c
 *
 *  Created on: May 20, 2022
 *      Author: nicth
 */

#include "stm32l475xx_gpio_driver.h"
/*
 * ***************************************************************************************************************************************
 *                                APIs supported by GPIO driver
 * ***************************************************************************************************************************************
 */
/*
 * **************************************************************************************************************************************
 *                                          Peripheral Clock setup
 ****************************************************************************************************************************************
 *
 *                  This function enables or disables peripheral clock for the given GPIO port
 *                  Param <base address of the GPIO peripheral><ENABLE or DISABLE macros>
 */
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi){
	if(EnorDi ==ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx ==GPIOD){
		    GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	}else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx ==GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
	}

}
/*
 * *******************************************************************************************************************************
 *                                                        Init - DeInit
 *********************************************************************************************************************************
 */
/*
 * GPIO_Init
 */
void GPIO_Init(GPIO_Handle_t* pGPIOHandle){
	uint32_t temp = 0; //temp. register
	// enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle -> pGPIOx, ENABLE);
	//1. configure the mode of gpio pin
if(pGPIOHandle -> GPIO_PinConf.GPIO_PinMode <=GPIO_MODE_ANALOG){
	// the non interrupt mode
	temp = (pGPIOHandle -> GPIO_PinConf.GPIO_PinMode << ( 2 * pGPIOHandle -> GPIO_PinConf.GPIO_PinNumber) ); //setting
	pGPIOHandle -> pGPIOx -> MODER &= ~( 0x3 << 2 * pGPIOHandle -> GPIO_PinConf.GPIO_PinNumber);//clearing
	pGPIOHandle -> pGPIOx -> MODER |= temp;
	temp = 0;
}else{
	//interrupt mode
	if(pGPIOHandle -> GPIO_PinConf.GPIO_PinMode >GPIO_MODE_IT_FT){
		                 // 1. configure the FTSR
		EXTI -> FTSR1 |= (1 << pGPIOHandle -> GPIO_PinConf.GPIO_PinNumber);
		                 // clear the corresponding RTSR bit
		EXTI -> RTSR1 &= ~(1 << pGPIOHandle -> GPIO_PinConf.GPIO_PinNumber);
	}else if(pGPIOHandle -> GPIO_PinConf.GPIO_PinMode >GPIO_MODE_IT_RT){
		                 // 1. configure the RTSR
		EXTI -> RTSR1 |= (1 << pGPIOHandle -> GPIO_PinConf.GPIO_PinNumber);
		                 // clear the corresponding FTSR bit
		EXTI -> FTSR1 &= ~(1 << pGPIOHandle -> GPIO_PinConf.GPIO_PinNumber);
	}else if(pGPIOHandle -> GPIO_PinConf.GPIO_PinMode >GPIO_MODE_IT_FRT){
		                 // 1. configure both FTSR and RTSR
		EXTI -> FTSR1 |= (1 << pGPIOHandle -> GPIO_PinConf.GPIO_PinNumber);
		EXTI -> RTSR1 |= (1 << pGPIOHandle -> GPIO_PinConf.GPIO_PinNumber);
	}
	                     // 2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t x = 0, PortCode = 0;
		PortCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle -> pGPIOx);
		x =  (pGPIOHandle -> GPIO_PinConf.GPIO_PinNumber / 4);
		SYSCFG_PCLK_EN();
		SYSCFG -> EXTICR[x] |= (PortCode <<  (pGPIOHandle -> GPIO_PinConf.GPIO_PinNumber % 4) * 4);

	                     // 3. enable the exti interrupt delivery using IMR
		EXTI -> IMR1 |= (1 << pGPIOHandle -> GPIO_PinConf.GPIO_PinNumber);
}
	//2. configure the speed
    temp = (pGPIOHandle -> GPIO_PinConf.GPIO_PinSpeed << (2 * pGPIOHandle -> GPIO_PinConf.GPIO_PinNumber ));
    pGPIOHandle -> pGPIOx -> OSPEEDR &= ~( 0x3 << 2 * pGPIOHandle -> GPIO_PinConf.GPIO_PinNumber);//clearing
    pGPIOHandle -> pGPIOx -> OSPEEDR |= temp;
	temp = 0;
	//3. configure the pupd settings
	temp = (pGPIOHandle -> GPIO_PinConf.GPIO_PinPuPdControl << (2 * pGPIOHandle -> GPIO_PinConf.GPIO_PinNumber ));
    pGPIOHandle -> pGPIOx -> PUPDR &= ~( 0x3 << 2 * pGPIOHandle -> GPIO_PinConf.GPIO_PinNumber);//clearing
	pGPIOHandle -> pGPIOx -> PUPDR |= temp;
	temp = 0;
	//4. configure optypes
	temp = (pGPIOHandle -> GPIO_PinConf.GPIO_PinOPType << ( pGPIOHandle -> GPIO_PinConf.GPIO_PinNumber));
	pGPIOHandle -> pGPIOx -> OTYPER &= ~( 0x3 << pGPIOHandle -> GPIO_PinConf.GPIO_PinNumber);//clearing
	pGPIOHandle -> pGPIOx -> OTYPER |= temp;

	temp = 0;
	//5. configure the alt functionality
	if(pGPIOHandle -> GPIO_PinConf.GPIO_PinMode == GPIO_MODE_ALTFN){
		//configure the alt function registers
		uint32_t temp1, temp2;
		temp1 = ( pGPIOHandle -> GPIO_PinConf.GPIO_PinNumber / 8);
		temp2 = ( pGPIOHandle -> GPIO_PinConf.GPIO_PinNumber % 8);

		pGPIOHandle -> pGPIOx -> AFRL[temp1] &= ~(0xF << (4 * temp2));//clearing
		pGPIOHandle -> pGPIOx -> AFRL[temp1] |= (pGPIOHandle -> GPIO_PinConf.GPIO_PinAltFunMode << ( 4 * temp2));
	}
}
/*
 * **********************************************************************************************************************************
 *                                                             GPIO_DeInit
 * ***********************************************************************************************************************************
  */
 /*
 *                                                        RESET GPIOx Peripherals
 */
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx){

		if(pGPIOx == GPIOA){
			GPIOA_REG_RESET();
		}else if(pGPIOx == GPIOB){
			GPIOB_REG_RESET();
		}else if(pGPIOx == GPIOC){
			GPIOC_REG_RESET();
		}else if(pGPIOx ==GPIOD){
		    GPIOD_REG_RESET();
		}else if(pGPIOx == GPIOE){
			GPIOE_REG_RESET();
		}else if(pGPIOx == GPIOF){
			GPIOF_REG_RESET();
		}else if(pGPIOx == GPIOG){
			GPIOG_REG_RESET();
		}else if(pGPIOx == GPIOH){
			GPIOH_REG_RESET();
	    }
}
/*
 * ************************************************************************************************************************************
 *                                                        GPIO Read From Input Pin
 * *************************************************************************************************************************************
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t * pGPIOx,uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx -> IDR >> PinNumber) & 0x00000001);
return value;
}
/*
 *
 * ****************************************************************************************************************************************
 *                                                      GPIO Read From Input Port
 * ****************************************************************************************************************************************
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t * pGPIOx){
	uint16_t value;
	value = (uint16_t)(pGPIOx -> IDR);
return value;
}
/*
 *
 * ****************************************************************************************************************************************
 *                                                      GPIO Write To Output Pin
 * ****************************************************************************************************************************************
 */
void GPIO_WriteToOuputPin(GPIO_RegDef_t * pGPIOx, uint8_t PinNumber,uint8_t Value){

	if(Value == GPIO_PIN_SET){
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx -> ODR |= (1 << PinNumber);
	}else{
		//write 0
		pGPIOx -> ODR &= ~(1 << PinNumber);
	}
}
/*
 *
 * ****************************************************************************************************************************************
 *                                                      GPIO Write To Output Port
 * ****************************************************************************************************************************************
 */
void GPIO_WriteToOuputPort(GPIO_RegDef_t * pGPIOx, uint16_t Value){
	pGPIOx -> ODR = Value;

}
/*
 *
 * ****************************************************************************************************************************************
 *                                                      GPIO Toggle Output Pin
 * ****************************************************************************************************************************************
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t * pGPIOx, uint8_t PinNumber){
		pGPIOx -> ODR ^= (1 << PinNumber);
}

/*
 * ****************************************************************************************************************************************
 *                                                  IRQ Configuration and ISR handling
 ******************************************************************************************************************************************
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			        // program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64){
			        // program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			       // program ISER2 register
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	}else{
		if(IRQNumber <= 31){
			        // program ICER0 register
			*NVIC_ICER0 &= ~(1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64){
			        // program ICER1 register
			*NVIC_ICER1 &= ~(1 << IRQNumber % 32);
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			        // program ICER2 register
			*NVIC_ICER2 &= ~(1 << IRQNumber % 64);
		}
	}
}
/*
 * ****************************************************************************************************************************************
 *                                                  IRQ Priority Configuration and ISR handling
 ******************************************************************************************************************************************
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
//1. First lets find out the IPR register
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_Section = IRQNumber % 4;
	uint8_t Shift_Amount = (8 * IPRx_Section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + IPRx) |= (IRQPriority << Shift_Amount);
}
/*
 *
 * ****************************************************************************************************************************************
 *                                                    GPIO IRQ Handling
 * ****************************************************************************************************************************************
 */
void GPIO_IRQHandling(uint8_t PinNumber){
//clear the exti PR register correspomding to the pin number
	if(EXTI -> PR1 & (1 << PinNumber)){
		//clear
		EXTI -> PR1 |= (1<<PinNumber);
	}
}


