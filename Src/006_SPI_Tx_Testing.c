/*
 *                                                                      006_SPI_Tx.c
 *
 */
#include <stm32l475xx.h>
#include <string.h>
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
	SPIPins.pGPIOx = GPIOA;

	SPIPins.GPIO_PinConf.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConf.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConf.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//SPI1_MISO
	SPIPins.GPIO_PinConf.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPIPins);
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
	SPI1Handle.pSPIx = SPI1;

	SPI1Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1Handle.SPIConfig.SPI_CPha = SPI_CPHA_LOW;
	SPI1Handle.SPIConfig.SPI_CPol = SPI_CPOL_LOW;
	SPI1Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1Handle.SPIConfig.SPI_SClkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI1Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;     //software slave management enabled for NSS pin

	SPI_Init(&SPI1Handle);
}
int main(void){
	char User_Data[] = "Hello World";
	//this function is used to initialize the GPIO Pins to behave as SPI1 pins
	SPI1_GPIOInits();
	//this fuction is used to initialize the SPI1 peripheral parameters
	SPI1_Inits();
	//SPI1_SSI config this make NSS signal internally high and avoid MODF error
	SPI_SSIConfig(SPI1,ENABLE);
	//enable SPI1 peripheral
	SPI_PeripheralControl(SPI1,ENABLE);
	//to send data
	SPI_SendData(SPI1, (uint8_t*)User_Data, strlen(User_Data));
	//disable SPI1 peripheral
	SPI_PeripheralControl(SPI1,DISABLE);
	while(1);
	return 0;
}
