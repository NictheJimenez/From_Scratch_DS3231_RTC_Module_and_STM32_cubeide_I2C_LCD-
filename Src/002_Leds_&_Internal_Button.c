/*
 *                                        002_leds_Internal_button.c
 */

#include "stm32l475xx.h"

#define LOW                                 0
#define BTN_PRESSED                         LOW

void delay(void){
	for(uint32_t i = 0; i < 5000; i++);
}

int main(void){


	GPIO_Handle_t GpioLedE, GpioLedI, GpioBtnI;
	/*
	 *  (GpioLedE => GPIO Led External from the board),
	 *  (GpioLedI => GPIO Led Internal from the board),
	 *  (GpioBtnI => GPIO Button Internal from the board)
	 */
	/************************External Led => GpioLedE*************************/
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
	/*************************Internal Led => GpioLedI*************************/
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
	/**********************Internal Button => GpioBtnI*************************/
	/* (Pin Name => PC13) (Signal or Label => BUTTON_EXTI13)
	 *( Feature / Comment => GPIO_EXTI13) (Connector => NA) (Pin number => NA)
	 *************************************************************************/
	GpioBtnI.pGPIOx = GPIOC;

	GpioBtnI.GPIO_PinConf.GPIO_PinNumber =GPIO_PIN_NO_13;
	GpioBtnI.GPIO_PinConf.GPIO_PinMode = GPIO_MODE_IN;
	//GpioBtnI.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PP;                 NA
	//GpioBtnI.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_FAST;                  NA
	GpioBtnI.GPIO_PinConf.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtnI);

	while(1){
        if((GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)) == BTN_PRESSED){

			GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_2);

			GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_14);
			delay();
        }
	}

	return 0;
}
