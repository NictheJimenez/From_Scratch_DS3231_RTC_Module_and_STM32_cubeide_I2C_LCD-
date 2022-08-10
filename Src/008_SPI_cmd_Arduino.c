/*
                                                                        * 008_SPI_TxRx_Arduino.c
 */

#include <stm32l475xx.h>
#include <string.h>
#include <stdio.h>
//command codes
#define COMMAND_LED_CTRL                    0X50
#define COMMAND_SENSOR_READ                 0X51
#define COMMAND_LED_READ                    0X52
#define COMMAND_PRINT                       0X53
#define COMMAND_ID_READ                     0X54

#define LED_ON                              1
#define LED_OFF                             0

//arduino analog pins
#define ANALOG_PIN0                         0
#define ANALOG_PIN1                         1
#define ANALOG_PIN2                         2
#define ANALOG_PIN3                         3
#define ANALOG_PIN4                         4

#define LED_PIN                             9

#define LOW                                 0
#define BTN_PRESSED                         LOW

void delay(void){
	for(uint32_t i = 0; i < 5000; i++);
}
/*
 * ********************************************************< SPI1 >*****************************************************************
 *                               Pin Name              Signal or Label                               Feature / Comment
 * SPI1_MISO                     PA6                   ARD.D12-SPI1_MISO                             SPI1_MISO
 * SPI1_MOSI                     PA7                   ARD.D11-SPI1_MOSI/PWM                         SPI1_MOSI
 * SPI1_SCLK                     PA5                   ARD.D13-SPI1_SCK/LED1                         SPI1_SCK
 * SPI1_NSS                      PA4                   ARD.D7                                        GPIO_Output
 * ALT_FUNCTION MODE   ->        5
 * **********************************************************************************************************************************
 */
void SPI1_GPIOInits(void){
	GPIO_Handle_t SPIPins;
	memset(&SPIPins,0,sizeof(SPIPins));												/*  (GpioLedI => GPIO Led Internal from the board),*/

	SPIPins.pGPIOx = GPIOA;

	SPIPins.GPIO_PinConf.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConf.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConf.GPIO_PinPuPdControl = GPIO_PU;
	SPIPins.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SPI1_SCLK
	SPIPins.GPIO_PinConf.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);
	//SPI1_MOSI
	SPIPins.GPIO_PinConf.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins);
	//SPI1_MISO
	SPIPins.GPIO_PinConf.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPIPins);
	//SPI1_NSS
	SPIPins.GPIO_PinConf.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPIPins);
}
void SPI1_Inits(void){
	SPI_Handle_t SPI1Handle;
	memset(&SPI1Handle,0,sizeof(SPI1Handle));
	SPI1Handle.pSPIx = SPI1;

	SPI1Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1Handle.SPIConfig.SPI_SClkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI1Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1Handle.SPIConfig.SPI_CPha = SPI_CPHA_LOW;
	SPI1Handle.SPIConfig.SPI_CPol = SPI_CPOL_LOW;
	SPI1Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;     //hardware slave management enable for NSS pin

	SPI_Init(&SPI1Handle);
}
void GPIO_ButtonInit(void){
	/**********************Internal Button => GpioBtnI*************************/
	/* (Pin Name => PC13) (Signal or Label => BUTTON_EXTI13)
	 *( Feature / Comment => GPIO_EXTI13) (Connector => NA) (Pin number => NA)
	 *************************************************************************/
	GPIO_Handle_t GpioBtnI;
	memset(&GpioBtnI,0,sizeof(GpioBtnI));
	GpioBtnI.pGPIOx = GPIOC;

	GpioBtnI.GPIO_PinConf.GPIO_PinNumber =GPIO_PIN_NO_13;
	GpioBtnI.GPIO_PinConf.GPIO_PinMode = GPIO_MODE_IN;
	//GpioBtnI.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PP;                 NA
	//GpioBtnI.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_FAST;                  NA
	GpioBtnI.GPIO_PinConf.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtnI);
}
uint8_t SPI_VerifyResponse(uint8_t ackbyte){
	if(ackbyte == 0xf5){
		//ack
		return 1;
	}
	//NACK
	return 0;
}
int main(void){
	uint8_t Dummy_Write = 0xf;
	uint8_t Dummy_Read;
	uint8_t ackbyte;
	//char User_Data[] = "Hello World\0";
	//this function is used to initialize the GPIO Pins to behave as SPI1 pins
	SPI1_GPIOInits();
	//this fuction is used to initialize the SPI1 peripheral parameters
	SPI1_Inits();
	//GPIO_ButtonInit();

	//SPI1_SSI config this make NSS signal internally high and avoid MODF error
	//SPI_SSIConfig(SPI1,ENABLE);

	//SPI1_SSOE config
	/*
	 * making SSOE 1 does NSS output enable. The NSS pin is automatically managed by the hardware. i.e when SPE=1, NSS will be pulled to low and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI1,ENABLE);
	while(1){
		//wait till button is pressed
		//while((GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)));
		//delay();

		//enable SPI1 peripheral
		SPI_PeripheralControl(SPI1,ENABLE);

		//1. CMD_LED_CTRL     <pin no(1)>            <value(1)>
		uint8_t commandcode = COMMAND_LED_CTRL;

		uint8_t args[2];
		//send command
		SPI_SendData(SPI1, &commandcode, 1);
		//do dummy read to clear off the RXNE flag
		SPI_ReciveData(SPI1, &Dummy_Read, 1);

		//send some dummy bits (1byte) to fetch the response from the slave.
		SPI_SendData(SPI1, &Dummy_Write, 1);
		//read the ack byte received
		SPI_ReciveData(SPI1, &ackbyte, 1);
	if(SPI_VerifyResponse(ackbyte)){
			//send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			//send arguments
			SPI_SendData(SPI1, args, 2);
		}
		//end of COMMAND_LED_CTRL
		/*
		//2.  CMD_SENSOR_READ           <analog pin number(1)>
		//wait till button is pressed
		while((GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)));
		delay();
		commandcode  =COMMAND_SENSOR_READ;
		//send command
		SPI_SendData(SPI1, &commandcode, 1);
		//do dummy read to clear off the RXNE flag
		SPI_ReciveData(SPI1, &Dummy_Read, 1);
		//send some dummy bits (1byte) to fetch the response from the slave.
		SPI_SendData(SPI1, &Dummy_Write, 1);
		//read the ack byte received
		SPI_ReciveData(SPI1, &ackbyte, 1);
		if(SPI_VerifyResponse(ackbyte)){
			//send arguments
			args[0] = ANALOG_PIN0;
			//sending one byte of
			SPI_SendData(SPI1, args, 1);
			//do dummy read to clear off the RXNE
			SPI_ReciveData(SPI1, &Dummy_Read, 1);
			//insert some delay so that slave can ready with the data
			delay();
			//send some dummy bits (1 byte)fetch the response from the slave
			SPI_SendData(SPI1, &Dummy_Write, 1);
			uint8_t analog_read;
			SPI_ReciveData(SPI1, &analog_read, 1);
		}
	    */

		while( SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG) );
		//Disable the SPI1 peripheral
        SPI_PeriClockControl(SPI1, DISABLE);
	  }
	return 0;
}

