/*
 *                                        001_Led_Toggle
 */
#include "stm32l475xx.h"

void delay(void){
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void){


	GPIO_Handle_t GpioLedE, GpioLedI;
	/***********************************External Led*************************/
	/* (Pin Name => PB2) (Signal or Label => ARD.D8)
	 *( Feature / Comment => GPIO_Output) (Connector => CN1) (Pin number => 1)
	 *************************************************************************/
	GpioLedE.pGPIOx = GPIOB;

	GpioLedE.GPIO_PinConf.GPIO_PinNumber =GPIO_PIN_NO_2;
	GpioLedE.GPIO_PinConf.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLedE.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLedE.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLedE.GPIO_PinConf.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioLedE);
	/***********************************Internal Led*************************/
	/* (Pin Name => PB14) (Signal or Label => LED2)
	 *( Feature / Comment => GPIO_Output) (Connector => NA) (Pin number => NA)
	 *************************************************************************/
	GpioLedI.pGPIOx = GPIOB;

	GpioLedI.GPIO_PinConf.GPIO_PinNumber =GPIO_PIN_NO_14;
	GpioLedI.GPIO_PinConf.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLedI.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLedI.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLedI.GPIO_PinConf.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioLedI);

	while(1){

		GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_2);
		GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_14);
		delay();
	}

	return 0;
}
/*void EXTI0_IRQHandler(void){
	//handle the interrupt
	GPIO_IRQHandling(0);
}*/
