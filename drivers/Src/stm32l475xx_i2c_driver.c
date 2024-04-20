/*
 *                                                                                stm32l475xx_i2c_drivers.c
 */

#include <stm32l475xx_i2c_driver.h>

// I2C timeout, about 2ms
#define I2C_TIMEOUT             200U
// Maximum NBYTES value
#define I2C_NBYTES_MAX          255U

// Definition of bits to reset in CR2 register
#define I2C_CR2_ALL            (I2C_CR2_SADD    | \
								I2C_CR2_NBYTES  | \
								I2C_CR2_RELOAD  | \
								I2C_CR2_AUTOEND | \
								I2C_CR2_RDWRN  | \
								I2C_CR2_START   | \
								I2C_CR2_STOP)

// Definition of all bits in ICR register (clear all I2C flags at once)
#define I2C_ICR_ALL            (I2C_ICR_ADDRCF  | \
								I2C_ICR_ALERTCF | \
								I2C_ICR_ARLOCF  | \
								I2C_ICR_BERRCF  | \
								I2C_ICR_NACKCF  | \
								I2C_ICR_OVRCF   | \
								I2C_ICR_PECCF   | \
								I2C_ICR_STOPCF  | \
								I2C_ICR_TIMOUTCF)
//0x20303e5d
/*
 * Generate Start Condition
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t * pI2Cx);

static void I2C_ExecuteAddressPhase(I2C_RegDef_t * pI2Cx, uint8_t SlaveAddr);

static void I2C_ClearADDRFlag(I2C_RegDef_t * pI2Cx);

static void I2C_ClearSTOPCFFlag(I2C_RegDef_t * pI2Cx);

static void I2C_ClearNACKCFFlag(I2C_RegDef_t * pI2Cx);

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

static void I2C_AddressMode7bits(I2C_RegDef_t *pI2Cx);

static void  I2C_TransferDirWriteMasterM(I2C_RegDef_t *pI2Cx);

static void  I2C_TransferDirReadMasterM(I2C_RegDef_t *pI2Cx);

static void I2C_TransferDirWorRMasterM(I2C_RegDef_t *pI2Cx,uint8_t RorWTransfer);

static void I2C_NBytestobeTransferred(I2C_RegDef_t *pI2Cx,uint8_t Len);

//static void I2C_NBytestoControl(I2C_RegDef_t *pI2Cx);

static void I2C_AutoendCondition(I2C_RegDef_t *pI2Cx, uint8_t AutoendEnorDi);

static void I2C_RestartCondition(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr,uint8_t RorWTransfer,uint8_t Len,uint8_t  AutoendEnorDi,uint8_t RestartdEnorDi);

static uint32_t I2C_CalcDelay(uint32_t delay);
/*
* Count rough delay for timeouts
*/
static uint32_t I2C_CalcDelay(uint32_t delay) {
	uint32_t cnt, SystemCoreClock = 16000000;;

	if (SystemCoreClock > 1000000U) {
		cnt = (delay * ((SystemCoreClock / 1000000U) + 1U));
	} else {
		cnt = (((delay / 100U) + 1U) * ((SystemCoreClock / 10000U) + 1U));
	}

	return cnt;
}
/*
 * ADD10: 10-bit addressing mode (master mode)
0: The master operates in 7-bit addressing mode
 */
static void I2C_AddressMode7bits(I2C_RegDef_t *pI2Cx){
	pI2Cx ->CR2 &= ~(1 << I2C_CR2_ADD10);
}

/*
 * RD_WRN: Transfer direction (master mode)
0: Master requests a write transfer.
1: Master requests a read transfer
 */
static void  I2C_TransferDirWriteMasterM(I2C_RegDef_t *pI2Cx){
	 // Master requests a write transfer.   (master mode)
		pI2Cx -> CR2 &= ~(1 << I2C_CR2_RDWRN);

}

static void  I2C_TransferDirReadMasterM(I2C_RegDef_t *pI2Cx){

		pI2Cx -> CR2 |= (1 << I2C_CR2_RDWRN);

}

static void I2C_TransferDirWorRMasterM(I2C_RegDef_t *pI2Cx,uint8_t RorWTransfer){
	if(RorWTransfer == I2C_WRITE_TRANSFER_MM){
		pI2Cx -> CR2 &= ~(1 << I2C_CR2_RDWRN);
	}else if(RorWTransfer == I2C_READ_TRANSFER_MM){
		pI2Cx -> CR2 |= (1 << I2C_CR2_RDWRN);
	}
}
/*
 *  Start generation
This bit is set by software, and cleared by hardware after the Start followed by the address
sequence is sent, by an arbitration loss, by a timeout error detection, or when PE = 0. It can
also be cleared by software by writing ‘1’ to the ADDRCF bit in the I2C_ICR register.
0: No Start generation.
1: Restart/Start generation:
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t * pI2Cx){

	pI2Cx -> CR2 |= (1 << I2C_CR2_START);

}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t * pI2Cx, uint8_t SlaveAddr){
	//pI2Cx -> CR2 &= ~( 0x7F << 1);
	//SlaveAddr = SlaveAddr >> 1;
	//SlaveAddr &= ~(1); // SlaveAddr is Slave Address + r/w bit = 0
	pI2Cx -> CR2 |= (SlaveAddr << 0 );
}

/*Number of bytes
*The number of bytes to be transmitted/received is programmed there. This field is don’t care
*in slave mode with SBC=0.
*Note: Changing these bits when the START bit is set is not allowed.
 */
static void I2C_NBytestobeTransferred(I2C_RegDef_t *pI2Cx,uint8_t Len){
	pI2Cx ->CR2 &= ~(0xFF << I2C_CR2_NBYTES);
	pI2Cx ->CR2 |= (Len << I2C_CR2_NBYTES);
}

static void I2C_ClearADDRFlag(I2C_RegDef_t * pI2Cx){
	uint32_t dummySet;

	dummySet = (1 << I2C_ICR_ADDRCF);
	pI2Cx -> ICR |= dummySet;
}

static void I2C_ClearSTOPCFFlag(I2C_RegDef_t * pI2Cx){
	uint32_t dummySet;

	dummySet = (1 << I2C_ICR_STOPCF);
	pI2Cx->ICR |= dummySet;
}

static void I2C_ClearNACKCFFlag(I2C_RegDef_t * pI2Cx){
	uint32_t dummySet;

	dummySet = (1 << I2C_ICR_NACKCF);
	pI2Cx->ICR |= dummySet;
}

static void I2C_AutoendCondition(I2C_RegDef_t *pI2Cx, uint8_t AutoendEnorDi){
	if (AutoendEnorDi == DISABLE) { // Master requests a write transfer.   (master mode)
		pI2Cx->CR2 &= ~(1 << I2C_CR2_AUTOEND); //1: Automatic end mode: a STOP condition is automatically sent when NBYTES data are transferred.
	} else if (AutoendEnorDi == ENABLE) {
		pI2Cx->CR2 |= (1 << I2C_CR2_AUTOEND); //0: software end mode: TC flag is set when NBYTES data are transferred, stretching SCL low.
	}
}

static void I2C_RestartCondition(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr,
		uint8_t RorWTransfer, uint8_t Len, uint8_t AutoendEnorDi,
		uint8_t RestartdEnorDi) {

	//Addressing mode (7-bit or 10-bit): ADD10 => 0 <=: 10-bit addressing mode (master mode)
	//0: The master operates in 7-bit addressing mode,
	//1: The master operates in 10-bit addressing mode
	I2C_AddressMode7bits(pI2Cx);

	//Slave address to be sent: SADD[9:0]
	//program the device address
	//: Slave address (master mode)
	//In 7-bit addressing mode (ADD10 = 0):
	//SADD[7:1] should be written with the 7-bit slave address to be sent. The bits SADD[9],
	//SADD[8] and SADD[0] are don't care.
	I2C_ExecuteAddressPhase(pI2Cx, SlaveAddr);

	//Transfer direction: RD_WRN
	I2C_TransferDirWorRMasterM(pI2Cx, RorWTransfer);

	//The number of bytes to be transferred: NBYTES[7:0]. If the number of bytes is equal to
	//or greater than 255 bytes, NBYTES[7:0] must initially be filled with 0xFF.
	I2C_NBytestobeTransferred(pI2Cx, Len);

	//In automatic end mode (AUTOEND=1), a STOP is automatically sent.
	//In software end mode (AUTOEND=0), the TC flag is
	I2C_AutoendCondition(pI2Cx, AutoendEnorDi);

	//I2C_NBytestoControl(pI2CHandle -> pI2Cx);
	// Start generation with not Bus busy
	//while((I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_BUSY)));
	// Start generation
	//This bit is set by software, and cleared by hardware after the Start followed by the address
	// sequence is sent, by an arbitration loss, by a timeout error detection, or when PE = 0. It can
	// also be cleared by software by writing ‘1’ to the ADDRCF bit in the I2C_ICR register.
	// 0: No Start generation.
	// 1: Restart/Start generation:
	I2C_GenerateStartCondition(pI2Cx);

}

/*
 *  Stop generation (master mode)
The bit is set by software, cleared by hardware when a STOP condition is detected, or when
PE = 0.
In Master Mode:
0: No Stop generation.
1: Stop generation after current byte transfer.
 */
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx -> CR2 &= ~(1 << I2C_CR2_STOP);
	pI2Cx -> CR2 |= (1 << I2C_CR2_STOP);
}

/*
 *
 */
/*static void I2C_NBytestoControl(I2C_RegDef_t *pI2Cx){
	pI2Cx -> CR1 &= ~(1 << I2C_CR1_SBC);
	pI2Cx -> CR2 &= ~(1 << I2C_CR2_RELOAD);
	pI2Cx -> CR1 |= (1 << I2C_CR1_SBC);
	pI2Cx -> CR2 |= (1 << I2C_CR2_RELOAD);
}
*/

// Clock configuration

//uint32_t RCC_PLLOutClk(){
//	 return;
 //}

  // Clock configuration

 //uint32_t RCC_MSIOutClk(){
//	 return;
// }

 /*
  * Clock configuration register (RCC_CFGR)
 */

void Timing_Settings(I2C_Handle_t *pI2CHandle){

	pI2CHandle ->pI2Cx ->TIMINGR &= ~(0xFF << I2C_TIMINGR_SCLL);
	pI2CHandle ->pI2Cx ->TIMINGR |= (0x5D << I2C_TIMINGR_SCLL);
   //pI2CHandle ->pI2Cx ->TIMINGR |= (0x13 << I2C_TIMINGR_SCLL);

	pI2CHandle ->pI2Cx ->TIMINGR &= ~(0xFF << I2C_TIMINGR_SCLH);
	pI2CHandle ->pI2Cx ->TIMINGR |= (0x3d << I2C_TIMINGR_SCLH);
	//pI2CHandle ->pI2Cx ->TIMINGR |= (0xF << I2C_TIMINGR_SCLH);

	pI2CHandle ->pI2Cx ->TIMINGR &= ~(0xF << I2C_TIMINGR_SDADEL);
	pI2CHandle ->pI2Cx ->TIMINGR |= (0x0 << I2C_TIMINGR_SDADEL);
	//pI2CHandle ->pI2Cx ->TIMINGR |= (0x2 << I2C_TIMINGR_SDADEL);

	pI2CHandle ->pI2Cx ->TIMINGR &= ~(0xF << I2C_TIMINGR_SCLDEL);
	pI2CHandle ->pI2Cx ->TIMINGR |= (0x3 << I2C_TIMINGR_SCLDEL);
	//pI2CHandle ->pI2Cx ->TIMINGR |= (0x4 << I2C_TIMINGR_SCLDEL);

	pI2CHandle ->pI2Cx ->TIMINGR &= ~(0xF << I2C_TIMINGR_PRESC);
	//pI2CHandle ->pI2Cx ->TIMINGR |= (3 << I2C_TIMINGR_PRESC);
	pI2CHandle ->pI2Cx ->TIMINGR |= (0x0 << I2C_TIMINGR_PRESC);
}

 /* Peripheral Clock Setup
 */
void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi){
	if (EnorDi == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}
	} else if (pI2Cx == I2C1) {
		I2C1_PCLK_DI();
	} else if (pI2Cx == I2C2) {
		I2C2_PCLK_DI();
	} else if (pI2Cx == I2C3) {
		I2C3_PCLK_DI();
	}
}
/*
 * Init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t tempreg = 0;
	I2C_PeripheralControl(pI2CHandle -> pI2Cx, DISABLE);
	//enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle ->pI2Cx, ENABLE);

	pI2CHandle -> pI2Cx -> CR1 &= ~(1 << I2C_CR1_ANFOFF);
	pI2CHandle -> pI2Cx -> CR1 &= ~(0x3 << I2C_CR1_DNF);

	pI2CHandle -> pI2Cx -> CR2 &= ~(1 << I2C_CR2_NACK);

	//pI2CHandle -> pI2Cx ->  ICR |= (1 << I2C_ICR_STOPCF);
	//pI2CHandle -> pI2Cx ->  ICR |= (1 << I2C_ICR_NACKCF);
	//ACK control bit
	/*tempreg = pI2CHandle ->I2C_Config.I2C_AckControl << I2C_CR2_NACK;//setting
	pI2CHandle -> pI2Cx -> CR2 &= ~(1 << I2C_CR2_NACK);//clearing
	pI2CHandle -> pI2Cx -> CR2 = tempreg;
*/


    //configure timings settings
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value();
	Timing_Settings(pI2CHandle);

	//I2C slave mode
	//I2C slave initialization
	// hi2c->Instance->OAR1 = (I2C_OAR1_OA1EN | hi2c->Init.OwnAddress1);

	tempreg = pI2CHandle -> I2C_Config.I2C_DeviceAddress << 1;
	pI2CHandle -> pI2Cx -> OAR1 &= ~(1 << I2C_OAR1_OA1EN);
	pI2CHandle -> pI2Cx -> OAR1 |= tempreg;
	tempreg = 0;
	tempreg = (1 << I2C_OAR1_OA1EN);
	pI2CHandle -> pI2Cx -> OAR1 &= ~tempreg;

	pI2CHandle -> pI2Cx -> OAR1 |= tempreg;

	//The general call address is enabled by setting the GCEN bit in the I2C_CR1 register.
	//
	//pI2CHandle -> pI2Cx -> CR1 &= ~(1 << I2C_CR1_GCEN);
	//pI2CHandle -> pI2Cx -> CR1 |= (1 << I2C_CR1_GCEN);

	pI2CHandle -> pI2Cx -> CR1 &= ~(1 << I2C_CR1_NOSTRETCH);
	//pI2CHandle -> pI2Cx -> CR1 |= (1 << I2C_CR1_NOSTRETCH);

	//allow byte ACK control in slave reception mode. Slave byte control mode must be enabled by setting the SBC bit in the I2C_CR1 register
	//Note: The SBC bit must be configured when the I2C is disabled, or when the slave is not
	//addressed, or when ADDR=1
	//pI2CHandle -> pI2Cx -> CR1 &= ~(1 << I2C_CR1_SBC);
	//pI2CHandle -> pI2Cx -> CR1 |= (1 << I2C_CR1_SBC);

	//Bit 6 TCIE: Transfer complete interrupt enable
	pI2CHandle -> pI2Cx -> CR1 &= ~(1 << I2C_CR1_TCIE);
	pI2CHandle -> pI2Cx -> CR1 &= ~(1 << I2C_CR1_NACKIE);
	pI2CHandle -> pI2Cx -> CR1 &= ~(1 << I2C_CR1_TXIE);

	//The user must then set the START bit in I2C_CR2 register. Changing all the above bits is
	//not allowed when START bit is set.
	pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR2_AUTOEND);
	//pI2CHandle -> pI2Cx -> CR2 &= ~(1 << I2C_CR2_STOP);
}
/*
 * De-Init
 */
void I2C_DeInit(I2C_RegDef_t* pI2Cx){
	//	I2C1-I2C2-I2C3 reset by APB1 peripheral reset register 1 (RCC_APB1RSTR1[1])
	if(pI2Cx == I2C1){
		I2C1_REG_RESET();
	}else if(pI2Cx == I2C2){
		I2C2_REG_RESET();
	}else if(pI2Cx == I2C3){
		I2C3_REG_RESET();
	}
}

/*
 * Data Send
 */
void I2C_MasterSendDataaux(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t RorWTransfer){

    //In order to initiate the communication, the user must program the following parameters for
	//the addressed slave in the I2C_CR2 register:

	//Addressing mode (7-bit or 10-bit): ADD10 => 0 <=: 10-bit addressing mode (master mode)
	//0: The master operates in 7-bit addressing mode,
	//1: The master operates in 10-bit addressing mode
	I2C_AddressMode7bits(pI2CHandle -> pI2Cx);

	//Slave address to be sent: SADD[9:0]
	//program the device address
	//: Slave address (master mode)
	//In 7-bit addressing mode (ADD10 = 0):
	//SADD[7:1] should be written with the 7-bit slave address to be sent. The bits SADD[9],
	//SADD[8] and SADD[0] are don't care.
	I2C_ExecuteAddressPhase(pI2CHandle -> pI2Cx, SlaveAddr);

	//Transfer direction: RD_WRN
     I2C_TransferDirWorRMasterM(pI2CHandle -> pI2Cx, RorWTransfer);

	//The number of bytes to be transferred: NBYTES[7:0]. If the number of bytes is equal to
	//or greater than 255 bytes, NBYTES[7:0] must initially be filled with 0xFF.
     I2C_NBytestobeTransferred(pI2CHandle -> pI2Cx, Len);
     //I2C_NBytestoControl(pI2CHandle -> pI2Cx);
     // Start generation with not Bus busy
     //while((I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_BUSY)));
    // Start generation
    //This bit is set by software, and cleared by hardware after the Start followed by the address
    // sequence is sent, by an arbitration loss, by a timeout error detection, or when PE = 0. It can
    // also be cleared by software by writing ‘1’ to the ADDRCF bit in the I2C_ICR register.
    // 0: No Start generation.
    // 1: Restart/Start generation:
     //I2C_ManageAcking(pI2CHandle -> pI2Cx, ENABLE);
     I2C_GenerateStartCondition(pI2CHandle -> pI2Cx);


	//6. Send the data until Len becomes 0
	//while( (I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_TCR)) ){
	//while(!(I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_TXE)));  // wait till TXE is set
	while(Len > 0  ){
		while((I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_TXIS)));  // wait till TXE is set
		pI2CHandle -> pI2Cx -> TXDR = * pTxBuffer;
		pTxBuffer ++;
		Len --;
	}


	//while(I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_TXE));
	while(I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_TC));
	//8. Generate Stop condition and master need not wait for the completion of Stop condition.
	// Note: generating STOP
	I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);
}
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t nbytes, uint8_t SlaveAddr, uint8_t AutoendEnorDi, uint8_t RestartdEnorDi){
	uint32_t delay_val = I2C_CalcDelay(I2C_TIMEOUT);
	register uint32_t reg;
	register uint32_t tx_count;
	register volatile uint32_t wait;

	//Clear all flags
	pI2CHandle -> pI2Cx -> ICR = I2C_ICR_ALL;

	// Everything regarding to the transmission is in the CR2 register
	reg = pI2CHandle -> pI2Cx -> CR2;
	reg &= ~I2C_CR2_ALL;

	// Whether it need to generate START condition
	if((pI2CHandle -> pI2Cx -> CR2 & (1 << I2C_CR2_START)) == DISABLE) {
		pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR2_START);
	}

		// Whether it need to generate STOP condition
	if ((nbytes  > I2C_NBYTES_MAX)){
		reg |= I2C_CR2_RELOAD;
	}

	// Transfer length
	tx_count = (nbytes  > I2C_NBYTES_MAX) ? I2C_NBYTES_MAX : nbytes;
	nbytes -= tx_count;

	//The number of bytes to be transferred: NBYTES[7:0]. If the number of bytes is equal to
	//or greater than 255 bytes, NBYTES[7:0] must initially be filled with 0xFF.
	I2C_NBytestobeTransferred(pI2CHandle -> pI2Cx, nbytes);

	//In order to initiate the communication, the user must program the following parameters for
	//the addressed slave in the I2C_CR2 register:
	//Addressing mode (7-bit or 10-bit): ADD10 => 0 <=: 10-bit addressing mode (master mode)
	//0: The master operates in 7-bit addressing mode,
	//1: The master operates in 10-bit addressing mode
	I2C_AddressMode7bits(pI2CHandle -> pI2Cx);
	//Slave address to be sent: SADD[9:0]
	//program the device address
	//: Slave address (master mode)
	//In 7-bit addressing mode (ADD10 = 0):
	//SADD[7:1] should be written with the 7-bit slave address to be sent. The bits SADD[9],
	//SADD[8] and SADD[0] are don't care.
	I2C_ExecuteAddressPhase(pI2CHandle -> pI2Cx, SlaveAddr);

	//Transfer direction: RD_WRN
	I2C_TransferDirWriteMasterM(pI2CHandle -> pI2Cx);

     //In automatic end mode (AUTOEND=1), a STOP is automatically sent.
     //In software end mode (AUTOEND=0), the TC flag is
    // I2C_AutoendCondition(pI2CHandle -> pI2Cx, AutoendEnorDi);


     //I2C_NBytestoControl(pI2CHandle -> pI2Cx);
     // Start generation with not Bus busy
     //while((I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_BUSY)));
    // Start generation
    //This bit is set by software, and cleared by hardware after the Start followed by the address
    // sequence is sent, by an arbitration loss, by a timeout error detection, or when PE = 0. It can
    // also be cleared by software by writing ‘1’ to the ADDRCF bit in the I2C_ICR register.
    // 0: No Start generation.
    // 1: Restart/Start generation:
     //I2C_GenerateStartCondition(pI2CHandle -> pI2Cx);
     uint8_t RorWTransfer = I2C_WRITE_TRANSFER_MM;
     //I2C_PeripheralControl(pI2CHandle -> pI2Cx, ENABLE);
	//6. Send the data until Len becomes 0
	//while( (I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_TCR)) ){
	//while(!(I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_TXE)));  // wait till TXE is set
 	//3. Enable the I2C peripheral
	while(tx_count  > 0 ){
		// Wait until either TXIS or NACK flag is set
		wait = delay_val;
		while (!((reg = pI2CHandle -> pI2Cx -> ISR) & ((1 << I2C_ISR_TXIS) | (1 << I2C_ISR_NACKF))) && --wait);
		if ((reg & (1 << I2C_ISR_NACKF)) || (wait == 0U)) {
			return 0;
		}

			while((I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_TXE)));  // wait till TXE is set
			pI2CHandle -> pI2Cx -> TXDR = * pTxBuffer;
			pTxBuffer ++;
			tx_count --;

			if ((tx_count == 0U) && (nbytes != 0U)) {
						// Wait until TCR flag is set (Transfer Complete Reload)
					wait = delay_val;
					while (!(pI2CHandle -> pI2Cx -> ISR & (1 << I2C_ISR_TCR)) && --wait);
					if (wait == 0U) {
						return 0;
					}
					// Configure next (or last) portion transfer
					reg = pI2CHandle -> pI2Cx -> CR2;
					reg &= ~((1 << I2C_CR2_NBYTES) | (1 << I2C_CR2_RELOAD) | (1 << I2C_CR2_AUTOEND));
					if ( (nbytes > I2C_NBYTES_MAX)) {
						reg |= (1 << I2C_CR2_RELOAD);
					} //else if (!(I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_STOPF))) {
						//reg |= (1 << I2C_CR2_AUTOEND);
					//}
					tx_count = (nbytes > I2C_NBYTES_MAX) ? I2C_NBYTES_MAX : nbytes;
						nbytes -= tx_count;
					//reg |= tx_count << I2C_CR2_NBYTES_Pos;
					//I2Cx->CR2 = reg;
	}
	//while(!(Len )){
		//if((pI2CHandle -> pI2Cx -> CR2 & (1 << I2C_CR2_RELOAD)) == DISABLE){
			//if((pI2CHandle -> pI2Cx -> CR2 & (1 << I2C_CR2_AUTOEND)) == DISABLE){

			//	while((I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_TC))){
			//		if(RestartdEnorDi == ENABLE){
			//			I2C_RestartCondition(pI2CHandle -> pI2Cx, SlaveAddr, RorWTransfer, Len,  AutoendEnorDi, RestartdEnorDi);
			//		}else{
						//8. Generate Stop condition and master need not wait for the completion of Stop condition.
						// Note: generating STOP
					//	I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);
			//		}
			//	}
			//}
		//}
	//}
}
}
/*
 *Data receive
*/
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t AutoendEnorDi, uint8_t RestartdEnorDi){

	//Addressing mode (7-bit or 10-bit): ADD10 => 0 <=: 10-bit addressing mode (master mode)
	//0: The master operates in 7-bit addressing mode,
	//1: The master operates in 10-bit addressing mode
	I2C_AddressMode7bits(pI2CHandle -> pI2Cx);

	//Slave address to be sent: SADD[9:0]
	//program the device address
	//: Slave address (master mode)
	//In 7-bit addressing mode (ADD10 = 0):
	//SADD[7:1] should be written with the 7-bit slave address to be sent. The bits SADD[9],
	//SADD[8] and SADD[0] are don't care.
	I2C_ExecuteAddressPhase(pI2CHandle -> pI2Cx, SlaveAddr);

	//Transfer direction: RD_WRN
	I2C_TransferDirReadMasterM(pI2CHandle -> pI2Cx);

     //The number of bytes to be transferred: NBYTES[7:0]. If the number of bytes is equal to
     //or greater than 255 bytes, NBYTES[7:0] must initially be filled with 0xFF.
    I2C_NBytestobeTransferred(pI2CHandle -> pI2Cx, Len);

     // Start generation
     //This bit is set by software, and cleared by hardware after the Start followed by the address
     // sequence is sent, by an arbitration loss, by a timeout error detection, or when PE = 0. It can
     // also be cleared by software by writing ‘1’ to the ADDRCF bit in the I2C_ICR register.
     // 0: No Start generation.
     // 1: Restart/Start generation:
     I2C_GenerateStartCondition(pI2CHandle -> pI2Cx);
     uint8_t RorWTransfer = I2C_READ_TRANSFER_MM;
		//read the data until Len becomes zero
     while(Len > 0 ){
			//wait until RXNE becomes 1
    		while((I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_RXNE))){}  // wait till RXNE is set

    		//read the data from data register in the buffer
    		 *pRxBuffer = pI2CHandle -> pI2Cx -> RXDR;
    		//if((pI2CHandle -> pI2Cx -> CR2 & (1 << I2C_CR2_AUTOEND)) == I2C_AUTOEND_DISABLE_MM){
    			//if((pI2CHandle -> pI2Cx -> CR2 & (1 << I2C_CR2_RELOAD)) == I2C_RELOAD_DISABLE_MM){

					if(Len == 0){
						//I2C_ISR.TC = 1?
						while(!(I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_TC)));  // wait till TC is set
						if(RestartdEnorDi == I2C_RESTART_ENABLE_MM){
							I2C_RestartCondition(pI2CHandle -> pI2Cx, SlaveAddr, RorWTransfer, Len,  AutoendEnorDi, RestartdEnorDi);
						}else
						//generate STOP condition
						I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);
					}
					//increment the buffer address
					pRxBuffer++;
					Len --;
    			//}
    		//}
		}

}
/*
 * I2C_MasterSendDataIT
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr){

	uint8_t busystate = pI2CHandle -> TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)){
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Addressing mode (7-bit or 10-bit): ADD10 => 0 <=: 10-bit addressing mode (master mode)
		//0: The master operates in 7-bit addressing mode,
		//1: The master operates in 10-bit addressing mode
		I2C_AddressMode7bits(pI2CHandle -> pI2Cx);

		//Slave address to be sent: SADD[9:0]
		//program the device address
		//: Slave address (master mode)
		//In 7-bit addressing mode (ADD10 = 0):
		//SADD[7:1] should be written with the 7-bit slave address to be sent. The bits SADD[9],
		//SADD[8] and SADD[0] are don't care.
		I2C_ExecuteAddressPhase(pI2CHandle -> pI2Cx, SlaveAddr);

		//Transfer direction: RD_WRN
		I2C_TransferDirWriteMasterM(pI2CHandle -> pI2Cx);

		//The number of bytes to be transferred: NBYTES[7:0]. If the number of bytes is equal to
		//or greater than 255 bytes, NBYTES[7:0] must initially be filled with 0xFF.
		I2C_NBytestobeTransferred(pI2CHandle -> pI2Cx, Len);
		//I2C_NBytestoControl(pI2CHandle -> pI2Cx);
		// Start generation with not Bus busy
		//while((I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_BUSY)));
		// Start generation
		//This bit is set by software, and cleared by hardware after the Start followed by the address
		// sequence is sent, by an arbitration loss, by a timeout error detection, or when PE = 0. It can
		// also be cleared by software by writing ‘1’ to the ADDRCF bit in the I2C_ICR register.
		// 0: No Start generation.
		// 1: Restart/Start generation:
		I2C_GenerateStartCondition(pI2CHandle -> pI2Cx);


		//I2C Interrupt requests Enable control bit

		//I2C_EV

		/* Address matched ADDR*/
		pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR1_ADDRIE);
		/*Receive buffer not empty RXNE*/
		pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR1_RXIE);
		/*Transmit buffer interrupt status TXIS*/
		pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR1_TXIE);
		/*Stop detection interrupt flag STOPF*/
		pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR1_STOPIE);
		/*Transfer complete reload TCR, Transfer complete TC*/
		pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR1_TCIE);
		/* NACK reception NACKF*/
		pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR1_NACKIE);

		//I2C_ER

		/* Enable: Bus error  BERR, Arbitration loss ARLO, Overrun/Underrun OVR, PEC error PECERR, Timeout/tLOW error TIMEOUT, SMBus alert ALERT*/
		pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR1_ERRIE);
	}

	return busystate;
}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr){
	uint8_t busystate = pI2CHandle -> TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)){
		pI2CHandle->pTxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Addressing mode (7-bit or 10-bit): ADD10 => 0 <=: 10-bit addressing mode (master mode)
		//0: The master operates in 7-bit addressing mode,
		//1: The master operates in 10-bit addressing mode
		I2C_AddressMode7bits(pI2CHandle -> pI2Cx);

		//Slave address to be sent: SADD[9:0]
		//program the device address
		//: Slave address (master mode)
		//In 7-bit addressing mode (ADD10 = 0):
		//SADD[7:1] should be written with the 7-bit slave address to be sent. The bits SADD[9],
		//SADD[8] and SADD[0] are don't care.
		I2C_ExecuteAddressPhase(pI2CHandle -> pI2Cx, SlaveAddr);

		//Transfer direction: RD_WRN
		I2C_TransferDirReadMasterM(pI2CHandle -> pI2Cx);

		//The number of bytes to be transferred: NBYTES[7:0]. If the number of bytes is equal to
		//or greater than 255 bytes, NBYTES[7:0] must initially be filled with 0xFF.
	    I2C_NBytestobeTransferred(pI2CHandle -> pI2Cx, Len);
		//I2C_NBytestoControl(pI2CHandle -> pI2Cx);
		// Start generation with not Bus busy
		//while((I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_BUSY)));
		// Start generation
		//This bit is set by software, and cleared by hardware after the Start followed by the address
		// sequence is sent, by an arbitration loss, by a timeout error detection, or when PE = 0. It can
		// also be cleared by software by writing ‘1’ to the ADDRCF bit in the I2C_ICR register.
		// 0: No Start generation.
		// 1: Restart/Start generation:

		I2C_GenerateStartCondition(pI2CHandle -> pI2Cx);

		//I2C Interrupt requests Enable control bit

		//I2C_EV

		/* Address matched ADDR*/
		pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR1_ADDRIE);
		/*Receive buffer not empty RXNE*/
		pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR1_RXIE);
		/*Transmit buffer interrupt status TXIS*/
		pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR1_TXIE);
		/*Stop detection interrupt flag STOPF*/
		pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR1_STOPIE);
		/*Transfer complete reload TCR, Transfer complete TC*/
		pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR1_TCIE);
		/* NACK reception NACKF*/
		pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR1_NACKIE);

		//I2C_ER

		/* Enable: Bus error  BERR, Arbitration loss ARLO, Overrun/Underrun OVR, PEC error PECERR, Timeout/tLOW error TIMEOUT, SMBus alert ALERT*/
		pI2CHandle -> pI2Cx -> CR2 |= (1 << I2C_CR1_ERRIE);
	}

	return busystate;
}
/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//1. First lets find out the IPR register
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_Section = IRQNumber % 4;
	uint8_t Shift_Amount = (8 * IPRx_Section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + IPRx) |= (IRQPriority << Shift_Amount);
}
/*
 * other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx,uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pI2Cx -> CR1 |= (1 << I2C_CR1_PE);
	}else{
		pI2Cx -> CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/*
 *
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint32_t FlagName){
	if(pI2Cx ->ISR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}
 /*
  * I2C_ManageAcking
  */
void I2C_ManageAcking(I2C_RegDef_t* pI2Cx,uint8_t EnorDi){
	if(EnorDi == I2C_ACK_ENABLE){
		//enable the ack
		pI2Cx -> CR2 &= ~(1 << I2C_CR2_NACK);
	}else{
		//disable the acking
		pI2Cx -> CR2 |= (1 << I2C_CR2_NACK);
	}
}
/*void I2C_SlaveSendData(I2C_Handle_t *pI2CHandle,uint8_t data){
	uint8_t temp = 0, tempcode = 0,tempdir = 0;
	//ADDR
	temp = pI2CHandle -> pI2Cx ->ISR  & (1 << I2C_ISR_ADDR);
	while(!temp);
	temp = 0;
	//Read ADDCODE and DIR in I2C_ISR
	//Set I2C_ICR.ADDRCF
	//EV1: ADDR ISR: check ADDCODE and DIR, set TXE, set ADDRCF
	tempcode = pI2CHandle -> pI2Cx ->ISR & (1 << I2C_ISR_ADDCODE);
	tempdir = pI2CHandle -> pI2Cx ->ISR  & (1 << I2C_ISR_DIR);
	temp = (uint8_t) pI2CHandle ->I2C_Config.I2C_DeviceAddress<< 1;
	//note: no entiendo el texto dir == T/R check
	if((tempcode == temp) && (tempdir ==  I2C_WRITE_TRANSFER_SM)){
        //Set I2C_ICR.ADDRCF
		pI2CHandle -> pI2Cx ->ICR |= (1 << I2C_ICR_ADDRCF);
		// wait till RXNE is set
		while(!(I2C_GetFlagStatus( pI2CHandle -> pI2Cx, I2C_FLAG_TXIS)));
		    		//write the data from data register in the buffer
		pI2CHandle -> pI2Cx ->TXDR = data;

	}
}*/

void I2C_SlaveReceiveData(I2C_Handle_t* pI2Cx, uint8_t *pRxBuffer){
	uint8_t temp = 0, tempcode = 0,tempdir = 0;
	//ADDR
	temp = pI2Cx ->pI2Cx ->ISR  & (1 << I2C_ISR_ADDR);
	while(!temp);
	temp = 0;
		//Read ADDCODE and DIR in I2C_ISR
		//Set I2C_ICR.ADDRCF
		//EV1: ADDR ISR: check ADDCODE and DIR, set TXE, set ADDRCF
		tempcode = pI2Cx ->pI2Cx ->ISR & (1 << I2C_ISR_ADDCODE);
		tempdir = pI2Cx ->pI2Cx ->ISR  & (1 << I2C_ISR_DIR);
		temp = (uint8_t) pI2Cx ->I2C_Config.I2C_DeviceAddress << 1;
		//note: no entiendo el texto dir == T/R check
		if((tempcode == temp) && (tempdir ==  I2C_READ_TRANSFER_SM)){
            //Set I2C_ICR.ADDRCF
			pI2Cx ->pI2Cx -> ICR |= (1 << I2C_ICR_ADDRCF);
			 // wait till RXNE is set
			while(! (I2C_GetFlagStatus( pI2Cx ->pI2Cx, I2C_FLAG_RXNE)) ){}
		    		//read the data from data register in the buffer
		    		*pRxBuffer =  (uint8_t) pI2Cx ->pI2Cx-> RXDR;
		}
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
	//Interrupt handling for both master and slave mode of a device
	uint8_t tempRXIE, tempTXIS, tempSTOPF, tempTCR, tempADDR, tempNACKF, tempFlag;

	tempRXIE = pI2CHandle  -> pI2Cx -> CR1 & (1 << I2C_CR1_RXIE);
	tempFlag = pI2CHandle -> pI2Cx -> ISR & (1 << I2C_ISR_RXNE);

	//1. handle for interrupt generated by RXNE event
	if(tempRXIE && tempFlag){
	// RXNE flag is set
	//Interrupt clear method Read I2C_RXDR register
		if(pI2CHandle -> TxRxState == I2C_BUSY_IN_RX){
			if(pI2CHandle ->RxLen > 0){
				// RXNE flag is set
				* pI2CHandle ->pRxBuffer = pI2CHandle -> pI2Cx -> RXDR;
				pI2CHandle ->pRxBuffer++;
				pI2CHandle ->RxLen--;
			}
		}
	}
	tempFlag = 0;

	tempTXIS = pI2CHandle  -> pI2Cx -> CR1 & (1 << I2C_CR1_TXIE);
	tempFlag = pI2CHandle -> pI2Cx -> ISR & (1 << I2C_ISR_TXIS);

	//1. handle for interrupt generated by TXIS event
	//Interrupt clear method Write I2C_TXDR register
	if(tempTXIS && tempFlag){
		if(pI2CHandle -> TxRxState == I2C_BUSY_IN_TX){
			if(pI2CHandle ->TxLen > 0){
				// TXNE flag is set
				pI2CHandle -> pI2Cx -> TXDR = * pI2CHandle ->pTxBuffer;
				pI2CHandle ->pTxBuffer++;
				pI2CHandle ->TxLen--;
			}
		}
	}
	tempFlag = 0;

	// STOPF flag is set
	//interrupt clear method Write STOPCF=1
	tempSTOPF = pI2CHandle  -> pI2Cx -> CR1 & (1 << I2C_CR1_STOPIE);
	tempFlag = pI2CHandle -> pI2Cx -> ISR & (1 << I2C_ISR_STOPF);
	//1. handle for interrupt generated by STOPF event
	if(tempSTOPF && tempFlag){
		// STOPF flag is set
		I2C_ClearSTOPCFFlag(pI2CHandle -> pI2Cx);
		//pI2CHandle -> pI2Cx -> ICR |= (1 << I2C_ICR_STOPCF);
	}
	tempFlag = 0;

	//interrupt clear method Write I2C_CR2 with NBYTES[7:0] ≠ 0
	// TCR/TC flag is set
	tempTCR = pI2CHandle  -> pI2Cx -> CR1 & (1 << I2C_CR1_TCIE);
	tempFlag = pI2CHandle -> pI2Cx -> ISR & (1 << I2C_ISR_TCR);

	//1. handle for interrupt generated by TCR event
	if(tempTCR && tempFlag){
		// TCR flag is set
		if(pI2CHandle -> TxRxState == I2C_BUSY_IN_TX){
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TCR_CMPLT);
		}
	}
	tempFlag = 0;

	//interrupt clear method Write START=1 or STOP=1
	// TCR/TC flag is set
	tempFlag = pI2CHandle -> pI2Cx -> ISR & (1 << I2C_ISR_TC);
	//1. handle for interrupt generated by TC event
	if(tempTCR && tempFlag){
		//check for device mode

		if(pI2CHandle -> TxRxState == I2C_BUSY_IN_TX){
			//if(pI2CHandle -> TxLen == 0){
			// TCR flag is set
			uint8_t temp1 =(pI2CHandle -> pI2Cx -> CR2 & (1 << I2C_CR2_AUTOEND)) == DISABLE;
			uint8_t temp2 = (pI2CHandle -> pI2Cx -> CR2 & (1 << I2C_CR2_RELOAD)) == DISABLE;
				if(temp1 && temp2){
					if(pI2CHandle -> TxLen == 0){
						// In this mode, the master automatically sends a STOP condition once the number of bytes programmed in the NBYTES[7:0] bit field is transferred
						I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);
						//reset all the member elements of the handle structure
						//I2C_CloseSendData();
						//notify the application about transmission complete
						I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
					}
				}
		}else if(pI2CHandle -> TxRxState == I2C_BUSY_IN_RX){
			if(pI2CHandle -> RxLen == 0){
				if( (pI2CHandle -> pI2Cx -> CR2 & (1 << I2C_CR2_AUTOEND)) == DISABLE && (pI2CHandle -> pI2Cx -> CR2 & (1 << I2C_CR2_RELOAD)) == DISABLE ){
					// In this mode, the master automatically sends a STOP condition once the number of bytes programmed in the NBYTES[7:0] bit field is transferred
					I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);
					//reset all the member elements of the handle structure
					//I2C_CloseSendData();
					//notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
				}
			}
		}
	}
	tempFlag = 0;

	//interrupt clear method Write ADDRCF=1
	//ADDR flag is set
	tempADDR = pI2CHandle  -> pI2Cx -> CR1 & (1 << I2C_CR1_ADDRIE);
	tempFlag = pI2CHandle -> pI2Cx -> ISR & (1 << I2C_ISR_ADDR);

	//1. handle for interrupt generated by ADDR event
	if(tempADDR && tempFlag){
	// ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle -> pI2Cx);
	}
	tempFlag = 0;

	//NACKF flag is set
	tempNACKF = pI2CHandle  -> pI2Cx -> CR1 & (1 << I2C_CR1_NACKIE);
	tempFlag = pI2CHandle -> pI2Cx -> ISR & (1 << I2C_ISR_NACKF);

	//1. handle for interrupt generated by NACKF event
	if(tempNACKF && tempFlag){
		// NACKF flag is set
		I2C_ClearNACKCFFlag(pI2CHandle -> pI2Cx);
	}


}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){
	uint8_t tempFlag, tempERRIE;

	//BERR flag is set
	tempERRIE = pI2CHandle  -> pI2Cx -> CR1 & (1 << I2C_CR1_ERRIE);
	tempFlag = pI2CHandle -> pI2Cx -> ISR & (1 << I2C_ISR_BERR);

	//1. handle for interrupt generated by Error
	if(tempERRIE && tempFlag){
	// BERR flag is set

	}

	// ARLO flag is set
	tempFlag = pI2CHandle -> pI2Cx -> ISR & (1 << I2C_ISR_ARLO);

	//1. handle for interrupt generated by Error
	if(tempERRIE && tempFlag){
	// ARLO flag is set

	}

	// OVR flag is set
	tempFlag = pI2CHandle -> pI2Cx -> ISR & (1 << I2C_ISR_OVR);
	if(tempERRIE && tempFlag){
	// OVR flag is set

	}

	// PECERR flag is set
	tempFlag = pI2CHandle -> pI2Cx -> ISR & (1 << I2C_ISR_PECERR);
	if(tempERRIE && tempFlag){
	// PECERR flag is set

	}

	// TIMEOUT flag is set
	tempFlag = pI2CHandle -> pI2Cx -> ISR & (1 << I2C_ISR_TIMEOUT);
	if(tempERRIE && tempFlag){
	// TIMEOUT flag is set

	}

	// ALERT flag is set
	tempFlag = pI2CHandle -> pI2Cx -> ISR & (1 << I2C_ISR_ALERT);
	if(tempERRIE && tempFlag){
	// ALERT flag is set

	}
}

