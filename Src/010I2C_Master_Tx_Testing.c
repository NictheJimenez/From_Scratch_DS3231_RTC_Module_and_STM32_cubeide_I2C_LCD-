/*
 *                                                                                         010I2C_Master_Tx_Testing.c
 */
#include <stm32l475xx.h>
#include <string.h>
#include <stdio.h>

I2C_Handle_t I2C1Handle;

// some data
uint8_t Some_Data[] = "Testing\n";

#define MY_ADDR                             0X61;
#define SLAVE_ADDR                          0X68
#define LOW                                 0
#define BTN_PRESSED                         LOW
void delay(void){
	for(uint32_t i = 0; i < 5000/2; i++);
}
/*
 * ********************************************************< I2C1 >*****************************************************************
 *                                       Pin Name               Signal or Label                              Feature / Comment
 *  I2C1_SCL                             PB8                    ARD.D15-I2C1_SCL                             I2C1_SCL
 *  I2C1_SDA                             PB9                    ARD.D14-I2C1_SDA                             I2C1_SDA
 * ALT_FUNCTION MODE  (AF4) ->        4
 * **********************************************************************************************************************************
 */
void I2C1_GPIOInits(void){
	GPIO_Handle_t I2CPins;
	memset(&I2CPins,0,sizeof(I2CPins));												/*  (GpioLedI => GPIO Led Internal from the board),*/

	I2CPins.pGPIOx = GPIOB;

	I2CPins.GPIO_PinConf.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConf.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConf.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConf.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	I2CPins.GPIO_PinConf.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//I2C1_SCL
	I2CPins.GPIO_PinConf.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO_Init(&I2CPins);
	//I2C1_SDA
	I2CPins.GPIO_PinConf.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);

}
void I2C1_Inits(void){

	memset(&I2C1Handle,0,sizeof(I2C1Handle));
	I2C1Handle.pI2Cx = I2C1;

	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
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

int main(void){
	//enable the i2c peripheral control

	//i2c pin inits
	I2C1_GPIOInits();


	I2C_PeriClockControl(I2C1, ENABLE);

	//i2c peripheral configuration
	I2C1_Inits();
	GPIO_ButtonInit();




	//send some data to the slave
	//I2C_MasterSendData(&I2C1Handle, Some_Data, strlen((char*)Some_Data), SLAVE_ADDR);


	while(1){
		//wait till button is press
	   // while((GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)) == BTN_PRESSED);
        //to avoid button de bouncing related issues 200ms of delay
		//delay();
		I2C_PeripheralControl(I2C1,ENABLE);
		//send some data to the slave
		I2C_MasterSendData(&I2C1Handle, Some_Data, strlen((char*)Some_Data), SLAVE_ADDR,I2C_WRITE_TRANSFER_MM);
		//I2C_MasterSendData(&I2C1Handle, Some_Data, strlen((char*)Some_Data), SLAVE_ADDR);

		while((I2C_GetFlagStatus(I2C1,  I2C_ISR_BUSY))){
					//disable SPI1 peripheral
					I2C_PeripheralControl(I2C1,DISABLE);
				}
	}
}


