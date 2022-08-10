/*
 *                                                                   007_SPI_TxOnly_Arduino.c
 */
#include <stm32l475xx.h>
#include <string.h>

#define LOW                                 0
#define BTN_PRESSED                         LOW

void delay(void){
	for(uint32_t i = 0; i < 500000/2; i++);
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
	SPIPins.GPIO_PinConf.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_LOW;

	//SPI1_MISO
	//SPIPins.GPIO_PinConf.GPIO_PinNumber = GPIO_PIN_NO_6;
	//GPIO_Init(&SPIPins);

	//SPI1_MOSI
	SPIPins.GPIO_PinConf.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins);
	//SPI1_SCLK
	SPIPins.GPIO_PinConf.GPIO_PinNumber = GPIO_PIN_NO_5;
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
	GpioBtnI.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_FAST;          //        NA
	GpioBtnI.GPIO_PinConf.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtnI);
}
int main(void){
	char User_Data[] = "Hello World";
	//int8_t pressed = 0;
	GPIO_ButtonInit();
	//this function is used to initialize the GPIO Pins to behave as SPI1 pins
	SPI1_GPIOInits();
	//this fuction is used to initialize the SPI1 peripheral parameters
	SPI1_Inits();
	GPIO_ButtonInit();

	//SPI1_SSI config this make NSS signal internally high and avoid MODF error
	//SPI_SSIConfig(SPI1,ENABLE);

	//SPI1_SSOE config
	/*
	 * making SSOE 1 does NSS output enable. The NSS pin is automatically managed by the hardware. i.e when SPE=1, NSS will be pulled to low and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI1,ENABLE);
	while(1){


		 while(!((GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)) == BTN_PRESSED)){}
			delay();

			//enable NSSP
		   // SPI_NSSPConfig(SPI1,ENABLE);
			//enable SPI1 peripheral
			SPI_PeripheralControl(SPI1,ENABLE);
			//first send length information
			uint8_t dataLen = 0;
			dataLen = strlen (User_Data);
			SPI_SendData(SPI1, &dataLen, 1);
			//to send data
			SPI_SendData(SPI1,(uint8_t*)User_Data, strlen(User_Data));

			while(SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG)){
				//disable SPI1 peripheral
				SPI_PeripheralControl(SPI1,DISABLE);
			}
	  }

	return 0;
}
