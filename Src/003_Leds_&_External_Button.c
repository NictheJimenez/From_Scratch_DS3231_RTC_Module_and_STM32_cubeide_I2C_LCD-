/*
 *                                                          003_Leds_External_Button.c
 */


#include "stm32l475xx.h"

#define LOW                                 0
#define BTN_PRESSED                         LOW

void delay(void){
	for(uint32_t i = 0; i < 500000 / 2; i++);
}

int main(void){


	GPIO_Handle_t GpioLedE, GpioLedI, GpioBtnE;
	/*
	 *  (GpioLedE => GPIO Led External from the board),
	 *  (GpioLedI => GPIO Led Internal from the board),
	 *  (GpioBtnE => GPIO Button External from the board)
	 */
	/************************External Led => GpioLedE*************************/
	/* (Pin Name => PB2) (Signal or Label => ARD.D8)
	 *( Feature / Comment => GPIO_Output) (Connector => CN1) (Pin number => 1)
	 *************************************************************************/
	GpioLedE.pGPIOx = GPIOB;

	GpioLedE.GPIO_PinConf.GPIO_PinNumber =GPIO_PIN_NO_2;
	GpioLedE.GPIO_PinConf.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLedE.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_OD;
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
	/**********************External Button => GpioBtnE*************************/
	/* (Pin Name => PD14) (Signal or Label =>  ARD.D2-INT0_EXTI14)
	 *( Feature / Comment => GPIO_EXTI14) (Connector => NA) (Pin number => NA)
	 *************************************************************************/
	GpioBtnE.pGPIOx = GPIOD;

	GpioBtnE.GPIO_PinConf.GPIO_PinNumber =GPIO_PIN_NO_14;
	GpioBtnE.GPIO_PinConf.GPIO_PinMode = GPIO_MODE_IN;
	//GpioBtnE.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PP;                 NA
	//GpioBtnE.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_FAST;                  NA
	GpioBtnE.GPIO_PinConf.GPIO_PinPuPdControl = GPIO_PU;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioBtnE);

	while(1){
        if((GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_14)) == BTN_PRESSED){
        	delay();
			GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_2);
			GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_14);
        }
	}

	return 0;
}
