/*
 *                                                                                stm32l475xx_i2c_drivers.c
 */

#include <stm32l475xx_i2c_driver.h>

// software to control the division factor of the AHB clock
uint16_t AHB_preScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
// software to control the division factor of the APB1 clock
uint16_t APB1_preScaler[4] = {2, 4, 8, 16};

/*
 * Generate Start Condition
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t * pI2Cx);

static void I2C_ExecuteAddressPhase(I2C_RegDef_t * pI2Cx, uint8_t SlaveAddr);

static void I2C_ClearADDRFlag(I2C_RegDef_t * pI2Cx);

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);


static void I2C_GenerateStartCondition(I2C_RegDef_t * pI2Cx){
	pI2Cx ->CR2 |= (1 << I2C_CR2_START);
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t * pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); // SlaveAddr is Slave Address + r/w bit = 0
}
static void I2C_ClearADDRFlag(I2C_RegDef_t * pI2Cx){
	uint32_t dummySet;

	dummySet = (1 << I2C_ICR_ADDRCF);
	pI2Cx -> ICR |= dummySet;
}
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){

	pI2Cx -> CR2 |= (1 << I2C_CR2_STOP);

}
/*
 * Clock configuration
 */
 uint32_t RCC_PLLOutClk(){
	 return;
 }
 /*
  * Clock configuration
  */
 uint32_t RCC_MSIOutClk(){
	 return;
 }
 /*
  * Clock configuration register (RCC_CFGR)
  */
uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp, ahbp, apbp1;

	clksrc = ((RCC ->CFGR >> 2) & 0x3);

	if(clksrc == RCC_CFGR_SWS_MSI){
		SystemClk = RCC_MSIOutClk();
	}else if(clksrc == RCC_CFGR_SWS_HSI){
		SystemClk = 16000000;
	}else if(clksrc == RCC_CFGR_SWS_HSE){
		SystemClk = 8000000;
	}else if(clksrc == RCC_CFGR_SWS_PLL){
		SystemClk = RCC_PLLOutClk();
	}

	//: AHB prescaler
	temp = ((RCC ->CFGR >> 4) & 0xf);

	if(temp < 8){
		ahbp = 1;
	}else{
		ahbp = AHB_preScaler[temp - 8];
	}

	//: APB1 prescaler
	temp = ((RCC ->CFGR >> 8) & 0x7);

	if(temp < 4){
		apbp1 = 1;
	}else{
		apbp1 = APB1_preScaler[temp - 4];
	}

	pclk1 = ((SystemClk / ahbp) / apbp1);

return pclk1;
}
void Timing_Settings(I2C_Handle_t *pI2CHandle){
	uint32_t temp = 0;

	temp = (3 << I2C_TIMINGR_PRESC );
	temp = (0x4 << I2C_TIMINGR_SCLDEL );
	temp = (0xF << I2C_TIMINGR_SCLH );
	temp = (0x13 << I2C_TIMINGR_SCLL );
	temp = (0x2 << I2C_TIMINGR_SDADEL );
	pI2CHandle ->pI2Cx ->TIMINGR  |= temp;
}
/*
 * Peripheral Clock Setup
 */
void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi){
	if(EnorDi ==ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	}else
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}else if(pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}
}
/*
 * Init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t tempreg = 0;

	//enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle ->pI2Cx, ENABLE);
	//ACK control bit
	tempreg |= pI2CHandle ->I2C_Config.I2C_AckControl << 15;
	pI2CHandle -> pI2Cx -> CR1 = tempreg;

    //configure timings settings
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value();
	Timing_Settings(pI2CHandle);

    //In order to initiate the communication, the user must program the following parameters for
	//the addressed slave in the I2C_CR2 register:

	//Addressing mode (7-bit or 10-bit): ADD10 => 0 <=: 10-bit addressing mode (master mode)
	//0: The master operates in 7-bit addressing mode,
	//1: The master operates in 10-bit addressing mode

	//Slave address to be sent: SADD[9:0]

	//Transfer direction: RD_WRN

	//The number of bytes to be transferred: NBYTES[7:0]. If the number of bytes is equal to
	//or greater than 255 bytes, NBYTES[7:0] must initially be filled with 0xFF.

	//program the device address
	//: Slave address (master mode)
	//In 7-bit addressing mode (ADD10 = 0):
	//SADD[7:1] should be written with the 7-bit slave address to be sent. The bits SADD[9],
	//SADD[8] and SADD[0] are don't care.
	tempreg |= pI2CHandle -> I2C_Config.I2C_DeviceAddress <<1;
	tempreg &= ~(1 << I2C_OAR1_OA1EN);
	pI2CHandle -> pI2Cx -> OAR1 |= tempreg;
	tempreg = 0;
	tempreg |= (1 << I2C_OAR1_OA1EN);
	pI2CHandle -> pI2Cx -> OAR1 |= tempreg;

	//The user must then set the START bit in I2C_CR2 register. Changing all the above bits is
	//not allowed when START bit is set.
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
 * Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr){
	//1. Generate the Start Condition
	I2C_GenerateStartCondition(pI2CHandle -> pI2Cx);
	//2. Confirm the Star generation is completed by checking the SB flag in the SR1
	//Note: Until SB is cleared SCL will Be stretched (pulled to low)
	//I2C_ClearStartandADDRFlag(pI2CHandle -> pI2Cx);
	//while(!(I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_SB)));
	//3.Send the address of the slave with r/w bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle -> pI2Cx, SlaveAddr);
	//4. Confirm that address phase is completed by checking the ADDR flag in the ISR
	while(! I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_ADDR));
	//5. Clear the ADDR flag according to its software sequence
	//Note: Until ADDR is Cleared SCL will be Stretched (pull to low)
	I2C_ClearADDRFlag(pI2CHandle -> pI2Cx);
	//6. Send the data until Len becomes 0
	while(Len > 0){
		while(I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_TXE));  // wait till TXE is set
		pI2CHandle -> pI2Cx -> TXDR = * pTxBuffer;
		pTxBuffer ++;
		Len --;
	}
	//7. When Len becomes zero wait for TXE = 1 and BTF = 1 before generating the Stop condition
	//Note: TXE = 1, TC = 1, means that DR are empty and next transmission should begin

	while(I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_TXE));
	while(I2C_GetFlagStatus(pI2CHandle -> pI2Cx, I2C_FLAG_TC));
	//8. Generate Stop condition and master need not wait for the completion of Stop condition.
	// Note: generating STOP
	I2C_GenerateStopCondition(pI2CHandle -> pI2Cx);
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
