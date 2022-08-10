/*
 *                                                                             009_SPI_Message_Rcv_It.c
 */
#include <stm32l475xx.h>
#include <string.h>
#include <stdio.h>
SPI_Handle_t SPI1Handle;

#define MAX_LEN                             500
char RcvBuff[MAX_LEN];
__vo uint8_t ReadByte;
__vo uint8_t RcvStop = 0;
//this flag will be set in the interrupt handler of the Arduino interrupt GPIO
__vo uint8_t DataAvailable = 0;

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

uint8_t SPI_VerifyResponse(uint8_t ackbyte){
	if(ackbyte == 0xf5){
		//ack
		return 1;
	}
	//NACK
	return 0;
}
/**********************Spi Init Pin => SpiInitPin*************************/
/* (Pin Name => PD14) (Signal or Label =>  ARD.D2-INT0_EXTI14)
 *( Feature / Comment => GPIO_EXTI14) (Connector => NA) (Pin number => NA)
 *************************************************************************/
void Slave_GPIO_InterruptPinInit(void){
	GPIO_Handle_t SpiInitPin;
	memset(&SpiInitPin,0,sizeof(SpiInitPin));
	SpiInitPin.pGPIOx = GPIOD;

	SpiInitPin.GPIO_PinConf.GPIO_PinNumber = GPIO_PIN_NO_14;
	SpiInitPin.GPIO_PinConf.GPIO_PinMode = GPIO_MODE_IT_FT;
	//SpiInitPin.GPIO_PinConf.GPIO_PinAltFunMode = 5;
	//SpiInitPin.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SpiInitPin.GPIO_PinConf.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SpiInitPin.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_LOW;

	GPIO_Init(&SpiInitPin);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI17);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

}
int main(void){
	uint8_t Dummy_Write = 0xf;
	uint8_t Dummy_Read;

	//this function is used to initialize the GPIO pins to behave as SPI1 pins
	Slave_GPIO_InterruptPinInit();
	//char User_Data[] = "Hello World\0";
	//this function is used to initialize the GPIO Pins to behave as SPI1 pins
	SPI1_GPIOInits();
	//this fuction is used to initialize the SPI1 peripheral parameters
	SPI1_Inits();
;

	//SPI1_SSI config this make NSS signal internally high and avoid MODF error
	//SPI_SSIConfig(SPI1,ENABLE);

	//SPI1_SSOE config
	/*
	 * making SSOE 1 does NSS output enable. The NSS pin is automatically managed by the hardware. i.e when SPE=1, NSS will be pulled to low and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI1,ENABLE);
	SPI_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
	while(1){

	RcvStop = 0;
	while(!DataAvailable);//wait till data available interrupt from transmitter device(slave)
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, DISABLE);
	//enable the SPI1 peripheral
	SPI_PeripheralControl(SPI1, ENABLE);
	while(!RcvStop){
		/*fetch the data from the SPI peripheral byte by byte in interrupt mode*/
		while(SPI_SendDataIT(&SPI1Handle, &Dummy_Write, 1) == SPI_BUSY_IN_TX);
		while(SPI_ReceiveDataIT(&SPI1Handle, &Dummy_Read, 1) == SPI_BUSY_IN_RX);
	}
	//confirm the SPI! is not busy
	while(SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG));
	//Disable the SPI1 peripheral
	SPI_PeriClockControl(SPI1,DISABLE);
	printf("Rcv data = %s\n",RcvBuff);
	DataAvailable = 0;
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, DISABLE);

	}
	return 0;
}
/*Runs when a data byte is received from the peripheral over SPI*/
void SPI1_IRQHandler(void){
	SPI_IRQHandling(&SPI1Handle);
}
/*SPI Application CallBack*/
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv){
	static uint32_t i = 0;
	/*In the Rx complete event, copy data in to rcv buffer. '\0' indicates end of the message (rcvStop = 1)*/
	if(AppEv == SPI_EVENT_RX_CMPLT){
		RcvBuff[i++] = ReadByte;
		if(ReadByte == '\0' || (i == MAX_LEN)){
			RcvStop = 1;
			RcvBuff[i-1] = '\0';
			i = 0;
		}
	}
}
void EXTI15_10_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_NO_14);
	DataAvailable = 1;
}


